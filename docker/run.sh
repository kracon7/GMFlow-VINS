#!/bin/bash

# Dockerfile builds a container with ROS and gnss_comm package.
# This file launches the container and bind GVINS source dir and 
# dataset dir with the container.
#
# The source dir is read only inside the container and changes
# from the host will show up inside the container. So develepment
# can be performed from the host side.
# 
# * After launching the container
# 1. run 'catkin build' to build the GVINS packages
# 2. 'source devel/setup.bash'
#
# * Enter the container from another terminal
# docker exec -it gvins /bin/bash

# IMPORTANT SETUP STEPS:
# In order to run RViz inside the container, the X server with 
# GPU must be properly setup.
# 1. The args in this files are necessary
# 2. Install nvidia-container-toolkit and follow steps below:
#    sudo nvidia-ctk runtime configure --runtime=docker
#    sudo systemctl restart docker

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
  -v $GVINS_DIR:/root/catkin_ws/src/GMFlow-VINS/:ro \
  -v $HOME/tmp:/root/tmp/ \
  gvins:latest \
  /bin/bash