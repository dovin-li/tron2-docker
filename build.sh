#!/bin/bash
# T14上执行: 构建Docker镜像 (不需要Mac)
set -e
cd ~/docker_build
docker build -t tron2-gazebo .
echo "=== 构建完成 ==="
echo "启动: bash ~/Desktop/docker_run.sh"
