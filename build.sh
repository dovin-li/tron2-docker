#!/bin/bash
# T14上执行: 构建Docker镜像
set -e
cd ~/docker_build
docker build -t tron2-gazebo .
docker image prune -f
echo "=== 构建完成 ==="
echo "启动: bash ~/start_tron2.sh"
