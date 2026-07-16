#!/bin/bash
# Mac上执行: 拷贝开源代码到T14并构建Docker镜像
set -e

T14="yhlee@192.168.10.34"
SRC="/Users/dovin_lee/Documents/代码/tron2开源"

echo "1/3 拷贝代码到T14..."
ssh $T14 "mkdir -p ~/docker_build/limxsdk-sim ~/docker_build/tron2_gazebo ~/docker_build/robot-description"
scp -r "$SRC/tron2_gazebo_ros/limxsdk-sim/"* $T14:~/docker_build/limxsdk-sim/
scp -r "$SRC/tron2_gazebo_ros/tron2_gazebo/"* $T14:~/docker_build/tron2_gazebo/
scp -r "$SRC/robot-description/"* $T14:~/docker_build/robot-description/

echo "2/3 拷贝Dockerfile..."
scp /tmp/docker/Dockerfile /tmp/docker/entry.sh $T14:~/docker_build/
scp -r /tmp/docker/robot_common $T14:~/docker_build/

echo "3/3 T14上构建镜像..."
ssh $T14 "cd ~/docker_build && docker build -t tron2-gazebo ."

echo "=== 完成 ==="
echo "启动命令: bash ~/Desktop/docker_run.sh"
