# tron2-rl-deploy-python 使用说明

## 1. 目录结构

- `main.py`：控制器入口程序（根据 `ROBOT_TYPE` 自动选择 SF/WF 控制器）。
- `controllers/SolefootController.py`：`SF_TRON2A` 推理与控制逻辑。
- `controllers/WheelfootController.py`：`WF_TRON2A` 推理与控制逻辑。
- `controllers/model/<ROBOT_TYPE>/`：每种机型的模型与配置文件目录。
- `limxsdk-lowlevel/`：LimX SDK 及示例代码。

## 2. 环境准备

### Step 1: 安装 Python 依赖

```bash
pip install -U pip
pip install numpy scipy pyyaml onnxruntime pygame
```

### Step 2: 安装 LimX SDK（必须）

按你的系统架构安装 wheel：

```bash
git clone https://github.com/limxdynamics/limxsdk-lowlevel.git

# x86_64 示例
pip install limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl

# aarch64 示例
pip install limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
```

## 3. 模型文件放置规则

模型文件需要按机型放在：

- `controllers/model/SF_TRON2A/policy.onnx`
- `controllers/model/SF_TRON2A/encoder.onnx`
- `controllers/model/SF_TRON2A/params.yaml`
- `controllers/model/WF_TRON2A/policy.onnx`
- `controllers/model/WF_TRON2A/encoder.onnx`
- `controllers/model/WF_TRON2A/params.yaml`

## 4. 运行控制器

### Step 1: 进入目录并设置机型

```bash
cd tron2-rl-deploy-python
export ROBOT_TYPE=SF_TRON2A
或 export ROBOT_TYPE=WF_TRON2A
```

### Step 2: 启动控制器

默认连接本机仿真（`127.0.0.1`）：

```bash
python3 main.py
```

指定机器人或 SDK 目标 IP：

```bash
python3 main.py 10.192.1.2
```

## 5. 与 MuJoCo 仿真联调

请确保仿真端和控制端使用相同的 `ROBOT_TYPE`：

- 仿真端：`tron2-mujoco-sim/simulator.py`
- 控制端：`tron2-rl-deploy-python/main.py`

建议先启动仿真，再启动控制器。

## 6. 手柄控制说明

- `L1 + Y`：切换到 WALK
- `L1 + X`：切回 IDLE
- `R1`：清空速度指令

- 打开一个 Bash 终端。

- 运行 robot-joystick：

  ```
  ./pointfoot-mujoco-sim/robot-joystick/robot-joystick
  ```

## 7. 效果展示

### 仿真部署 (Simulation)

![SF Simulation](doc/sfmj-ezgif.com-video-to-gif-converter.gif)
![WF Simulation](doc/wfmj-ezgif.com-video-to-gif-converter.gif)

### 实机部署 (Real-world)

实机部署时请悬挂启动控制器

![Deploy](doc/deploy.jpg)

## 8. 常见问题

- `ROBOT_TYPE not set`：先执行 `export ROBOT_TYPE=...`
- `Model not found`：检查 `controller/model/<ROBOT_TYPE>/` 下文件是否齐全
- `No module named limxsdk`：SDK wheel 未安装到当前 Python 环境
- `RobotState has not been received yet`：通常是仿真器未启动，或两端 `ROBOT_TYPE` 不一致

## 9. License

[Apache 2.0](LICENSE)
