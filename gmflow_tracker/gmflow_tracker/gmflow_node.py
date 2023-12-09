import os
import time
import torch
import torch.nn.functional as F
import logging
import yaml
import rospy
from indexed import IndexedOrderedDict
import numpy as np
from pathlib import Path

import PIL
from sensor_msgs.msg import Image, PointCloud
from .gmflow.gmflow import GMFlow
from .gmflow_evaluate import FlowPredictor

HOME = Path.home()

log = logging.getLogger("MOTION_FIELD")
 
class MotionFieldPublisher:
    def __init__(self, to_frame='base'):
        super().__init__('motion_field_publisher')

        self.camera_k = None
        self.camera_rays = None
        param_file = rospy.get_param('config_file')
        with open(param_file, 'r') as f:
            ros_params = yaml.load(f, Loader=yaml.Loader)

        self.feature_pub = rospy.Publisher('feature',
                                           PointCloud,
                                           queue_size=1)

        self.color_sub = rospy.Subscriber(ros_params['image_topic'], 
                                          Image, 
                                          self.image_callback,
                                          queue_size=1)
        self.frame_buffer = IndexedOrderedDict()

        # Load GMFlow network parameters
        gmflow_dir_root = os.path.dirname(
                                os.path.dirname(
                                    os.path.abspath('__file__')))
        gmflow_param_file = os.path.join(
                                gmflow_dir_root, 
                                'config',
                                'gmflow_with_refinement.yaml')
        with open(gmflow_param_file, 'r') as f:
            gmflow_params = yaml.load(f, Loader=yaml.Loader)

        # Init GMFlow and load checkpoint
        if 'LOCAL_RANK' not in os.environ:
            os.environ['LOCAL_RANK'] = str(gmflow_params['local_rank'])

        torch.backends.cudnn.benchmark = True
        device = torch.device('cuda:{}'.format(gmflow_params['local_rank']))
        self.device = device

        # model
        model = GMFlow(feature_channels=gmflow_params['feature_channels'],
                       num_scales=gmflow_params['num_scales'],
                       upsample_factor=gmflow_params['upsample_factor'],
                       num_head=gmflow_params['num_head'],
                       attention_type=gmflow_params['attention_type'],
                       ffn_dim_expansion=gmflow_params['ffn_dim_expansion'],
                       num_transformer_layers=gmflow_params['num_transformer_layers'],
                       ).to(device)

        num_params = sum(p.numel() for p in model.parameters())
        print('Number of params:', num_params)
    
        # resume checkpoints
        if gmflow_params['checkpoint'] != '':
            checkpoint_path = os.path.join(
                gmflow_dir_root,
                gmflow_params['checkpoint']
            )
            print('Load checkpoint: %s' % checkpoint_path)

        loc = 'cuda:{}'.format(gmflow_params['local_rank'])
        checkpoint = torch.load(checkpoint_path, map_location=loc)
        weights = checkpoint['model'] if 'model' in checkpoint else checkpoint

        model.load_state_dict(weights, strict=gmflow_params['strict_resume'])

        self.flow_predictor = FlowPredictor(model,
                                            None,
                                            gmflow_params['padding_factor'],
                                            gmflow_params['attn_splits_list'],
                                            gmflow_params['corr_radius_list'],
                                            gmflow_params['prop_radius_list'],
                                            gmflow_params['pred_bidir_flow'])

        self.frame_count = 0
        # Throw frame every
        self.frame_skip = 2
        # Run inference every k frames from buffer
        self.inference_every = 3
        # Inference between current frame and previous frame
        self.inference_skip = 2

        self.init_grid()

    def image_callback(self, color_msg: Image, wrist_depth_msg: Image):
        # Wait until we have a camera k
        if self.camera_k is None:
            return
        
        self.frame_count += 1
        if self.frame_count % self.frame_skip == 0:
            return
        
        now = time.time()
        print("Received frame id: %d"%self.frame_count)
        
        msg_time = color_msg.header.stamp.to_sec()

        wrist_depth = self.ros_depth_to_torch_tensor(wrist_depth_msg)
        wrist_points = wrist_depth * self.camera_rays
        wrist_rgb = self.ros_color_to_torch_tensor(color_msg)
        
        rgbxyz = torch.cat([wrist_rgb, wrist_points], dim=0)

        if self.to_frame == 'base':
            current_ja = np.array(self.last_joint_state.position[:7])
            T = torch.tensor(self.camera_pose_calculator.compute_robot_T_camera(
                                                            current_ja),
                            dtype=torch.float32, device=self.device)
        elif self.to_frame == 'camera':
            T = torch.eye(4, dtype=torch.float32, device=self.device)
            
        self.frame_buffer[msg_time] = {'rgbxyz': rgbxyz, 'T': T}

        if (self.frame_count // self.frame_skip) % self.inference_every != 0:
            return
        
        if len(self.frame_buffer) < 5:
            return

        while len(self.frame_buffer) > 10:
            self.frame_buffer.popitem(last=False)

        prev_idx = -1 - self.inference_skip
        next_idx = -1
        rgbxyz_1 = self.frame_buffer.values()[prev_idx]['rgbxyz']
        rgbxyz_2 = self.frame_buffer.values()[next_idx]['rgbxyz']
        dt = self.frame_buffer.keys()[next_idx] - self.frame_buffer.keys()[prev_idx]

        rgb_1 = rgbxyz_1[:3]
        rgb_2 = rgbxyz_2[:3]

        flow = self.flow_predictor.pred(rgb_1, rgb_2, self.finger_mask)
        flow = flow.permute((2,0,1)).unsqueeze(0)

        # Normalize flow for grid sampling
        flow[:,0] /= (self.im_h / 2)
        flow[:,1] /= (self.im_w / 2)

        pts_1 = F.grid_sample(rgbxyz_1[3:].unsqueeze(0), self.grid, mode='nearest', align_corners=True).reshape(3, -1).T
        vec_2d = F.grid_sample(flow, self.grid, mode='nearest', align_corners=True)
        grid_next = self.grid + vec_2d.permute(0,2,3,1)
        pts_2 = F.grid_sample(rgbxyz_2[3:].unsqueeze(0), grid_next, mode='nearest', align_corners=True).reshape(3, -1).T

        if self.to_frame == "base":
            print('TO BASE!!!!!')
            h_pts_1 = torch.cat([pts_1, 
                                 torch.ones([pts_1.shape[0],1], dtype=torch.float32, device=self.device)], dim=1)
            h_pts_2 = torch.cat([pts_2, 
                                 torch.ones([pts_1.shape[0],1], dtype=torch.float32, device=self.device)], dim=1)
            robot_T_cam_1 = self.frame_buffer.values()[prev_idx]['T']
            robot_T_cam_2 = self.frame_buffer.values()[next_idx]['T']
            h_pts_1 = (robot_T_cam_1 @ h_pts_1.T).T
            h_pts_2 = (robot_T_cam_2 @ h_pts_2.T).T
            pts_1 = h_pts_1[:,:3]
            pts_2 = h_pts_2[:,:3]

        vec = pts_2 - pts_1
        velocity = vec / dt
        vec_norm = torch.norm(vec, dim=1)
        # Pass filter and validility check
        valid = torch.logical_not(torch.isnan(velocity[:,0])) \
                & (vec_norm < 0.03) \
                & (vec_norm > 0.007)

        valid_pts = pts_1[valid]
        valid_vel = velocity[valid]

        mfield_msg = MotionField()
        mfield_msg.header = color_msg.header
        mfield_msg.step = 6
        mfield_msg.data = torch.cat([valid_pts, valid_vel], dim=1).reshape(-1).tolist()
        self.mfield_pub.publish(mfield_msg)
        print("GMFlow Node, image callback time: %.3fms"%(1e3 * (time.time() - now)))
    
    def ros_color_to_torch_tensor(self, color_msg: Image):
        torch_img = torch.frombuffer(color_msg.data, dtype=torch.uint8).to(self.device)
        torch_img = torch_img.reshape(color_msg.height, color_msg.width, 3).float()
        torch_img = torch_img[:,:,[2,1,0]]
        torch_img = torch_img.permute((2,0,1))
        return torch_img

    def init_grid(self):
        # Horizontal direction in image frame
        u = torch.linspace(-0.85, 0.85, 45, dtype=torch.float32, device=self.device)
        # Vertical direction in image frame
        v = torch.linspace(-0.85, 0.85, 35, dtype=torch.float32, device=self.device)
        vv, uu = torch.meshgrid(v, u, indexing='xy')
        self.grid = torch.stack([uu, vv], dim=-1).unsqueeze(0)

def main():
    rclpy.init()
    motion_field_publisher =MotionFieldPublisher()
    rclpy.spin(motion_field_publisher)
    motion_field_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()