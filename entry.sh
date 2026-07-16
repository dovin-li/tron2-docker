#!/bin/bash
source /opt/ros/noetic/setup.sh
source /ws/install/setup.bash
rosrun rosbridge_server rosbridge_websocket _port:=9090 &
rosrun tron2_gazebo ground_truth_odom.py &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_Link &
rosrun tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_imu mid360_Link &
exec "$@"
