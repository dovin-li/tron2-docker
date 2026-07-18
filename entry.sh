#!/bin/bash
source /opt/ros/noetic/setup.sh
source /ws/install/setup.bash
rosrun tron2_gazebo ground_truth_odom.py &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
rosrun tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_imu mid360_Link &
sleep 5
exec "$@"
# v2
# v3
