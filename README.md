# TRON2 Gazebo 仿真 Docker 镜像

基于 Docker 构建的 TRON2 四足机器人 Gazebo 仿真环境，在 Docker 容器内运行 **ROS1 Noetic + Gazebo 11**，通过 ROS1-ROS2 桥接与宿主机 **ROS2 Humble** 通信，实现完整的仿真、SLAM 与导航链路。

---

## 架构概览

```
┌──────────────────────────────────────────────────────────────────┐
│                    宿主机 (Ubuntu 22.04)                          │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                  ROS2 Humble                                │  │
│  │                                                             │  │
│  │  bridge_min.py ──→ bracket_filter.py ──→ densecloud_accum.  │  │
│  │                          │                    │             │  │
│  │                          ▼                    ▼             │  │
│  │                   /scan3d_filtered    octomap_server        │  │
│  │                          │                    │             │  │
│  │                          └────────┬───────────┘             │  │
│  │                                   ▼                         │  │
│  │                           slam_sim.py (RViz)                │  │
│  └────────────────────────────────────────────────────────────┘  │
│                            │ Docker bridge (--net host)          │
│                            ▼                                     │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │          Docker 容器 (Ubuntu 20.04 + nvidia)                │  │
│  │                                                             │  │
│  │  ROS1 Noetic + Gazebo 11                                    │  │
│  │  ┌──────────┐  ┌──────────────┐  ┌──────────────────────┐   │  │
│  │  │ gzserver │  │  gzclient    │  │ robot_state_publisher │   │  │
│  │  └──────────┘  └──────────────┘  └──────────────────────┘   │  │
│  │  ┌──────────┐  ┌──────────────┐  ┌──────────────────────┐   │  │
│  │  │tron2_hw  │  │ joy_node +   │  │   rosbridge          │   │  │
│  │  │  _node   │  │ joy_to_cmdvel│  │   _websocket         │   │  │
│  │  └──────────┘  └──────────────┘  └──────────────────────┘   │  │
│  │                                                             │  │
│  │  传感器: 2D/3D LiDAR + D435 Depth Camera + IMU + GT Odom   │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

- **宿主机**: Ubuntu 22.04, ROS2 Humble, NVIDIA 驱动, Docker + nvidia-container-toolkit
- **容器内**: Ubuntu 20.04, ROS1 Noetic, Gazebo 11, TRON2 机器人模型与控制栈
- **通信方式**: `--net host` 模式，容器内外共享网络，ROS1/ROS2 topic 通过 bridge 节点转发

---

## 目录结构

```
tron2-docker/
├── Dockerfile                  # Docker 镜像构建文件
├── build.sh                    # 一键构建脚本（含 docker prune）
├── entry.sh                    # 容器入口脚本（环境初始化 + 已知问题修复）
├── start_tron2.sh              # 宿主机启动脚本（清理 → 容器 → bridge → SLAM）
├── limxsdk-sim/                # LimX SDK 仿真接口库（ROS1）
├── onnxruntime_sdk/            # ONNX Runtime 推理 SDK（模型推理支持）
├── robot_common/               # 机器人公共定义（消息、服务、工具函数）
├── limxsdk-lowlevel/           # LimX 底层 SDK 封装（硬件抽象层）
├── robot-description/          # TRON2 URDF/XACRO 机器人模型描述
├── tron2_gazebo/               # Gazebo 仿真启动与配置
│   ├── launch/                 #   launch 文件（gazebo, spawn_urdf 等）
│   ├── worlds/                 #   Gazebo 世界文件
│   └── scripts/                #   ground_truth_odom.py 等脚本
├── tron2_controllers/          # ROS Control 控制器配置与实现
├── tron2_hw/                   # 硬件接口节点（tron2_hw_node, 手柄映射, Web 遥控）
│   ├── launch/                 #   tron2_hw_sim.launch 主启动文件
│   └── scripts/                #   joy_to_cmdvel.py, web_walk 等
└── README.md                   # 本文件
```

### 各目录用途说明

| 目录 | 用途 |
|------|------|
| `robot_common` | 机器人通用消息定义、服务接口、公共工具函数，为其他包提供基础数据结构 |
| `robot-description` | TRON2 机器人的 URDF/XACRO 模型描述，包含所有连杆、关节、传感器挂载位置和 TF 树定义 |
| `tron2_gazebo` | Gazebo 仿真启动配置（启动 gzserver/gzclient、spawn 模型、世界配置）以及真值里程计脚本 |
| `tron2_hw` | 硬件接口节点（仿真硬件抽象、手柄遥操作 `joy_to_cmdvel.py`、Web 遥控 `web_walk`）、主 launch 文件 |
| `tron2_controllers` | ROS Control 控制器配置，定义关节控制器类型、PID 参数等 |
| `limxsdk-sim` | LimX Dynamics 官方 SDK 的仿真适配层，提供仿真环境下的机器人通信与控制接口 |
| `limxsdk-lowlevel` | LimX 底层 SDK，封装底层硬件驱动和通信协议 |
| `onnxruntime_sdk` | ONNX Runtime 推理 SDK 集成，支持在仿真环境中部署训练好的 ONNX 模型 |

---

## 依赖清单

### 宿主机依赖

| 依赖 | 版本/说明 |
|------|-----------|
| **操作系统** | Ubuntu 22.04 (Jammy) |
| **ROS2** | ROS2 Humble Hawksbill |
| **NVIDIA 驱动** | 595+ (支持 Blackwell RTX 5060 等新架构) |
| **Docker** | Docker Engine 24+ |
| **nvidia-container-toolkit** | 启用 GPU 加速容器 |
| **图形环境** | X11 (需 `DISPLAY=:0`) |

### Docker 镜像内依赖

| 依赖 | 来源/说明 |
|------|-----------|
| **基础镜像** | `ros:noetic-ros-core-focal` |
| **Gazebo** | `gazebo11`, `libgazebo11-dev` |
| **ROS-Gazebo 桥接** | `ros-noetic-gazebo-ros`, `ros-noetic-gazebo-ros-control`, `ros-noetic-gazebo-plugins` |
| **ROS Control** | `ros-noetic-ros-control`, `ros-noetic-ros-controllers`, `ros-noetic-hardware-interface`, `ros-noetic-controller-manager`, `ros-noetic-controller-interface`, `ros-noetic-joint-state-controller` |
| **ROS 工具** | `ros-noetic-robot-state-publisher`, `ros-noetic-xacro`, `ros-noetic-realtime-tools`, `ros-noetic-rviz` |
| **手柄** | `ros-noetic-joy` |
| **导航** | `ros-noetic-move-base`, `ros-noetic-amcl`, `ros-noetic-gmapping`, `ros-noetic-map-server`, `ros-noetic-dwa-local-planner` |
| **WebSocket** | `ros-noetic-rosbridge-server` |
| **数学库** | `libeigen3-dev`, `liburdfdom-dev` |
| **编译工具** | `build-essential`, `python3-catkin-tools` |
| **Python** | `python3-pygame`, `python3-flask` |
| **Gazebo 传感器插件** | `libgazebo_ros_laser.so` (2D LiDAR), `libgazebo_ros_block_laser.so` (3D LiDAR), `libgazebo_ros_depth_camera.so` (深度相机), `libgazebo_ros_p3d.so` (真值位姿) |

---

## 构建步骤

### 方法一：直接 docker build

```bash
cd ~/docker_build
docker build -t tron2-gazebo .
```

### 方法二：使用 build.sh 脚本（推荐）

```bash
bash build.sh
```

`build.sh` 脚本会自动执行：
1. 清理旧的构建缓存和悬空镜像（`docker system prune -f`）
2. 执行 `docker build -t tron2-gazebo .`
3. 构建完成后输出镜像信息

**构建时间**：首次构建约 15-30 分钟（取决于网络速度），后续增量构建更快。

---

## 启动步骤

### 一键启动

```bash
bash ~/start_tron2.sh
```

### 启动脚本流程详解

`start_tron2.sh` 按以下顺序执行：

```bash
# 第一阶段：清理
killall -9 rviz2 2>/dev/null || true       # 关闭旧的 RViz2
pkill -9 -f bridge_min                     # 关闭旧的 ROS2 bridge
pkill -9 -f bracket_filter                 # 关闭旧的点云过滤器
pkill -9 -f densecloud_accumulator         # 关闭旧的稠密点云累积器
pkill -9 -f octomap_server                 # 关闭旧的 OctoMap 服务器
pkill -9 -f slam_sim                       # 关闭旧的 SLAM 可视化
sleep 1

docker rm -f $(docker ps -aq) 2>/dev/null  # 清理所有旧容器

# 第二阶段：启动 Docker 容器
DISPLAY=:0 xhost +local:docker              # 允许 Docker 访问 X11
docker run -d --rm \
  --net host \                              # 共享宿主机网络
  --runtime=nvidia --gpus all \             # GPU 加速
  --device /dev/input \                    # 手柄设备
  -e DISPLAY=:0 \                          # X11 显示
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name tron2_gazebo \
  tron2-gazebo \
  roslaunch tron2_hw tron2_hw_sim.launch robot_type:=SF_TRON2A

sleep 25  # 等待 Gazebo 完全启动

# 第三阶段：启动 ROS2 桥接与 SLAM
source /opt/ros/humble/setup.bash

python3 ~/ros2_slam/bridge_min.py &         # ROS1 → ROS2 topic 桥接
sleep 5

python3 ~/ros2_slam/bracket_filter.py &     # 点云过滤（按角度/距离裁剪）
sleep 2

python3 ~/ros2_slam/densecloud_accumulator.py &  # 稠密点云累积
sleep 2

ros2 run octomap_server octomap_server_node \    # OctoMap 建图
  --ros-args --params-file ~/ros2_slam/octomap_params.yaml \
  -r cloud_in:=/scan3d_filtered &
sleep 2

DISPLAY=:0 ros2 launch ~/ros2_slam/slam_sim.py & # RViz2 SLAM 可视化
```

### 容器内启动的节点

`tron2_hw_sim.launch` 在容器内启动以下 ROS1 节点：

| 节点 | 功能 |
|------|------|
| `gzserver` | Gazebo 仿真服务端（物理引擎 + 传感器） |
| `gzclient` | Gazebo 图形客户端 |
| `spawn_urdf` | 将 TRON2 URDF 模型加载到 Gazebo 世界 |
| `robot_state_publisher` | 发布 TF 变换（URDF 模型状态） |
| `tron2_hw_node` | TRON2 硬件接口节点（仿真硬件抽象） |
| `controller_spawner` | 加载并启动关节控制器 |
| `hold_base_in_air` | 保持机器人基座悬空（避免初始跌落） |
| `joy_node` | Xbox 手柄驱动节点 |
| `joy_to_cmdvel` | 手柄摇杆 → `/cmd_vel` 速度指令转换 |
| `web_walk` | Web 遥控界面（Flask） |
| `rosbridge_websocket` | WebSocket 桥接（供 Web 前端使用） |

---

## URDF 结构与 TF 树

### 完整 TF 链路

```
map
 │
 └── odom
      │
      └── base_footprint
           │
           └── base_link
                ├── base_imu
                ├── rack_link
                │    └── mid360_link
                ├── d435_Link
                │    └── d435_optical_frame
                ├── LF_HIP → LF_THIGH → LF_CALF → LF_FOOT
                ├── RF_HIP → RF_THIGH → RF_CALF → RF_FOOT
                ├── LH_HIP → LH_THIGH → LH_CALF → LH_FOOT
                ├── RH_HIP → RH_THIGH → RH_CALF → RH_FOOT
                ├── BODY_LEG_L0 (支腿) → ... → BODY_LEG_R0 (支腿)
                └── (共 10 组腿部关节链)
```

### 关键关节参数

#### rack_joint（Mid-360 LiDAR 支架）
```
origin: xyz=[0, 0, 0.2615], rpy=[0, 0, 0]
父连杆: base_imu → 子连杆: rack_link
```

#### mid360_joint（Mid-360 LiDAR 传感器，倒装安装）
```
origin: xyz=[0, 0, 0.19], rpy=[3.14159, 0, 0]
父连杆: rack_link → 子连杆: mid360_link
```
> **注意**: `rpy=[3.14159, 0, 0]` 表示 LiDAR 绕 X 轴旋转 180°（倒装安装），激光扫描平面朝向下方。

#### d435_Link（Intel RealSense D435 深度相机）
```
父连杆: base_link → 子连杆: d435_Link → d435_optical_frame
```

### TF 静态发布

容器启动时自动执行：
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link
```

---

## 传感器清单

### Mid-360 2D LiDAR（点云/激光扫描）

| 参数 | 值 |
|------|-----|
| **Gazebo 插件** | `libgazebo_ros_laser.so` |
| **ROS Topic** | `/scan` |
| **消息类型** | `sensor_msgs/LaserScan` |
| **发布频率** | 10 Hz |
| **采样点数** | 1800 samples/scan |
| **测距范围** | 0.1 m ~ 40 m |
| **扫描角度** | 360° |

### Mid-360 3D LiDAR（多线激光雷达模拟）

| 参数 | 值 |
|------|-----|
| **Gazebo 插件** | `libgazebo_ros_block_laser.so` |
| **ROS Topic** | `/scan3d` |
| **消息类型** | `sensor_msgs/PointCloud2` |
| **发布频率** | 10 Hz |
| **分辨率** | 180 水平采样 × 32 垂直射线 |
| **用途** | 3D SLAM (通过 bridge 转发至 ROS2 做 OctoMap 建图) |

### Intel RealSense D435 深度相机

| 参数 | 值 |
|------|-----|
| **Gazebo 插件** | `libgazebo_ros_depth_camera.so` |
| **输出** | 深度图像 + RGB 图像 + 点云 |
| **TF 挂载点** | `d435_Link` → `d435_optical_frame` |

### IMU（惯性测量单元）

| 参数 | 值 |
|------|-----|
| **挂载点** | `base_imu`（固定于 `base_link`） |
| **输出** | 姿态、角速度、线加速度 |

### Ground Truth Odometry（真值里程计）

| 参数 | 值 |
|------|-----|
| **Gazebo 插件** | `libgazebo_ros_p3d.so` |
| **辅助脚本** | `ground_truth_odom.py`（tron2_gazebo 软件包） |
| **功能** | 发布 `odom → base_footprint` 的地面真值位姿变换 |
| **用途** | 评估 SLAM 精度、导航定位基准 |

---

## ROS1 话题列表

容器内通过 `roslaunch tron2_hw tron2_hw_sim.launch` 启动后，发布以下 ROS1 话题：

| 话题 | 消息类型 | 发布者 | 说明 |
|------|----------|--------|------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (mid360 2D) | 2D 激光扫描，10Hz，360° |
| `/scan3d` | `sensor_msgs/PointCloud2` | Gazebo (mid360 3D) | 3D 点云，10Hz |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher + P3D | 动态坐标系变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | static_transform_publisher | 静态坐标系变换 |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | 仿真时钟 |
| `/cmd_vel` | `geometry_msgs/Twist` | joy_to_cmdvel / web_walk | 速度控制指令 |
| `/joy` | `sensor_msgs/Joy` | joy_node | Xbox 手柄原始数据 |
| `/joint_states` | `sensor_msgs/JointState` | controller_spawner | 关节状态（位置、速度、力矩） |
| `/ground_truth_odom` | `nav_msgs/Odometry` | ground_truth_odom.py | 地面真值里程计 |
| `/rosout` | `rosgraph_msgs/Log` | rosout | 日志聚合 |
| `/rosout_agg` | `rosgraph_msgs/Log` | rosout | 聚合日志 |

> 这些话题通过 `bridge_min.py` 桥接到宿主机 ROS2 网络，供 SLAM 和导航节点订阅。

---

## 手柄控制

### 硬件要求

- **Xbox 控制器**（Xbox One / Xbox Series 无线或有线）
- 手柄通过 USB 或蓝牙连接到宿主机

### 控制链路

```
Xbox 手柄 (/dev/input/js0)
        │
        ▼
   joy_node (ros-noetic-joy)
        │  发布 /joy (sensor_msgs/Joy)
        ▼
 joy_to_cmdvel.py (tron2_hw)
        │  摇杆值 → 速度映射
        ▼
    /cmd_vel (geometry_msgs/Twist)
        │
        ▼
  tron2_hw_node → Gazebo 机器人运动
```

### Docker 启动参数

必须在 `docker run` 时挂载宿主机输入设备：

```bash
--device /dev/input
```

`start_tron2.sh` 已包含此参数。如果容器内读不到手柄，检查：
1. 手柄是否已连接：`ls /dev/input/js*`
2. 权限：`sudo chmod a+rw /dev/input/js0`
3. 容器是否以 `--device /dev/input` 启动

---

## 已知问题

### 1. RTX 5060 Blackwell 架构 gzclient 崩溃

**现象**: 在 RTX 5060 (Blackwell 架构) 显卡上，Gazebo 11 客户端 (`gzclient`) 启动后立即崩溃或渲染异常。

**原因**: Gazebo 11 的 OGRE 渲染引擎与新一代 Blackwell GPU 存在兼容性问题。

**解决方案**: `entry.sh` 在容器启动时自动写入如下配置：

```bash
mkdir -p /root/.gazebo
cat > /root/.gazebo/gui.ini << GUIEOF
[geometry]
x=0
y=0
[rendering]
force_system_render=0
GUIEOF
```

`force_system_render=0` 会禁用系统级渲染优化，使用兼容模式。此修复已集成在 `entry.sh` 中，无需手动干预。

### 2. Cycle Time 偶尔超阈值

**现象**: Gazebo 控制台偶尔输出类似警告：
```
Warning: cycle time 0.004s exceeds threshold 0.002s
```

**影响**: 不影响仿真功能和传感器数据精度。该警告源于 Gazebo 物理引擎在复杂接触计算时的瞬时负载波动。

**处理**: 无需处理，可忽略。如频繁出现，可适当降低 Gazebo 实时更新率 (`real_time_update_rate`) 或提高最大步长 (`max_step_size`)。

### 3. P3D bodyName 大小写 FATAL 警告

**现象**: Gazebo 启动时输出：
```
[FATAL] P3DPlugin: bodyName [base_Link] not found
```

**原因**: `libgazebo_ros_p3d.so` 插件配置中的 `bodyName` 大小写与 URDF 中实际连杆名称不匹配（`base_Link` vs `base_link`）。

**影响**: 不影响主功能。该 P3D 插件实例会加载失败，但主要的里程计通过 `ground_truth_odom.py` 脚本提供，功能完整。

**处理**: 可忽略。如需修复，在 URDF/XACRO 中统一 `bodyName` 大小写即可。

---

## Git 仓库

- **仓库地址**: [github.com/dovin-li/tron2-docker](https://github.com/dovin-li/tron2-docker)
- **克隆命令**:
  ```bash
  git clone https://github.com/dovin-li/tron2-docker.git ~/docker_build
  ```

---

## 快速参考

```bash
# 克隆仓库
git clone https://github.com/dovin-li/tron2-docker.git ~/docker_build

# 构建镜像
cd ~/docker_build && bash build.sh

# 启动仿真
bash ~/start_tron2.sh

# 查看容器日志
docker logs -f tron2_gazebo

# 进入容器
docker exec -it tron2_gazebo bash

# 停止仿真
docker stop tron2_gazebo
```

---

## 环境变量

| 变量 | 值 | 说明 |
|------|-----|------|
| `DISPLAY` | `:0` | X11 显示输出 |
| `NVIDIA_DRIVER_CAPABILITIES` | `all` | 启用全部 NVIDIA 功能 |
| `NVIDIA_VISIBLE_DEVICES` | `all` | 所有 GPU 可见 |

---

## 相关资源

- [LimX Dynamics 官网](https://www.limxdynamics.com/)
- [TRON2 产品页面](https://www.limxdynamics.com/tron2)
- [ROS Noetic 文档](https://wiki.ros.org/noetic)
- [Gazebo 11 文档](https://classic.gazebosim.org/tutorials?tut=ros_overview&cat=connect_ros)
- [NVIDIA Container Toolkit 安装指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
