#!/bin/bash
# T14上执行: 启动ROS1 Gazebo Docker
xhost +local:docker 2>/dev/null
docker run -it --rm --gpus all \
  --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  tron2-gazebo \
  roslaunch tron2_gazebo empty_world.launch robot_type:=SF_TRON2A
