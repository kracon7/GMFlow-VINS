GVINS_DIR=$HOME/code/catkin_ws/src/GMFlow-VINS

echo $GVINS_DIR

xhost +local:docker 

docker run \
  -it \
  --rm \
  --net=host \
  --privileged \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=QT_X11_NO_MITSHM=1 \
  --gpus all \
  --name gvins \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
  -v $GVINS_DIR:/root/catkin_ws/src/GMFlow-VINS/ \
  -v $HOME/tmp:/root/tmp/ \
  gvins:latest \
  /bin/bash