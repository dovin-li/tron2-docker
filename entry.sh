#!/bin/bash
source /opt/ros/noetic/setup.sh
source /ws/install/setup.bash

# Fix for RTX 5060 Blackwell: force_system_render causes scene crash
mkdir -p /root/.gazebo
cat > /root/.gazebo/gui.ini << 'GUIEOF'
[geometry]
x=0
y=0
[rendering]
force_system_render=0
GUIEOF

rosrun tron2_gazebo ground_truth_odom.py &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
sleep 5
exec "$@"
