#!/root/.envs/py38/bin/python3.8

import os
import time
import torch
import torch.nn.functional as F
import logging
import yaml
import rospy
import numpy as np
from pathlib import Path

from sensor_msgs.msg import Image
from gvins_feature_tracker.srv import (
    EstimateGMFlow, 
    EstimateGMFlowRequest, 
    EstimateGMFlowResponse
)
from gmflow.gmflow import GMFlow
from gmflow_evaluate import FlowPredictor
from scipy.interpolate import interpn

HOME = Path.home()
 
class GMFlowEstimator:
    def __init__(self):
        self.im_h = None
        self.im_w = None

        # Load GMFlow network parameters
        gvins_dir_root = os.path.dirname(
                            os.path.dirname(
                                os.path.dirname(
                                    os.path.abspath(__file__))))
        gmflow_param_file = os.path.join(
                                gvins_dir_root, 
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
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
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

        self.init_grid()

    def estimate(self, req: EstimateGMFlowRequest):
        
        now = time.time()
        rospy.loginfo("Received request to estimate GMFlow")
        
        msg_time = req.img1.header.stamp.to_sec()
        rgb_1 = self.ros_color_to_torch_tensor(req.img1) # [2, H, W]
        rgb_2 = self.ros_color_to_torch_tensor(req.img2)
        cur_pts = np.stack([req.cur_pts.x, req.cur_pts.y]).T
        rospy.loginfo("Got %d cur_pts"%(cur_pts.shape[0]))

        flow = self.flow_predictor.pred(rgb_1, rgb_2, None) # [H, W, 2]

        if self.im_h is None:
            self.im_h = rgb_1.shape[1]
            self.im_w = rgb_1.shape[2]

        candidate_forw_pts = self.grid.reshape(-1, 2).cpu().numpy()
        np.random.shuffle(candidate_forw_pts)

        np_flow = flow.permute(1,0,2).cpu().numpy()  # W x H x 2
        axs = (np.arange(np_flow.shape[0]), np.arange(np_flow.shape[1]))
        # if cur_pts.shape[0] > 0:
        #     import pdb
        #     pdb.set_trace()
        flow_samples = interpn(axs, np_flow, cur_pts)
        forw_pts = cur_pts + flow_samples

        rospy.loginfo("Estimated %d forw_pts"%(forw_pts.shape[0]))

        res = EstimateGMFlowResponse()
        res.response.forw_pts.x = forw_pts[:, 0]
        res.response.forw_pts.y = forw_pts[:, 1]
        res.response.status = [1] * forw_pts.shape[0]
        res.response.candidate_forw_pts.x = candidate_forw_pts[:,0] * self.im_w
        res.response.candidate_forw_pts.y = candidate_forw_pts[:,1] * self.im_h

        rospy.loginfo("GMFlow Server, service callback time: %.3fms"%(1e3 * (time.time() - now)))
        return res
    
    def ros_color_to_torch_tensor(self, color_msg: Image):
        torch_img = torch.frombuffer(color_msg.data, dtype=torch.uint8).to(self.device)
        torch_img = torch_img.reshape(color_msg.height, color_msg.width, 3).float()
        torch_img = torch_img[:,:,[2,1,0]]
        torch_img = torch_img.permute((2,0,1))
        return torch_img

    def init_grid(self):
        # Horizontal direction in image frame
        u = torch.linspace(0.05, 0.95, 45, dtype=torch.float32, device=self.device)
        # Vertical direction in image frame
        v = torch.linspace(0.05, 0.95, 35, dtype=torch.float32, device=self.device)
        vv, uu = torch.meshgrid(v, u, indexing='xy')
        self.grid = torch.stack([uu, vv], dim=-1).unsqueeze(0)

def main():
    rospy.init_node('estimate_gmflow_server')
    gmflow_estimator = GMFlowEstimator()
    s = rospy.Service('estimate_gmflow', EstimateGMFlow, gmflow_estimator.estimate)
    print("Ready to estimate GMFlow")
    rospy.spin()


if __name__ == '__main__':
    print("Cuda device count: %d"%torch.cuda.device_count())
    main()
