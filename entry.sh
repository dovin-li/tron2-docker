#!/bin/bash
source /opt/ros/noetic/setup.sh
source /ws/install/setup.bash

# start rosbridge (WebSocket gateway for ROS2 host)
rosrun rosbridge_server rosbridge_websocket _port:=9090 &
# start ground_truth_odom (odom->base_footprint TF)
rosrun tron2_gazebo ground_truth_odom.py &
# start static TF base_footprint->base_Link
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_Link &

exec "$@"
