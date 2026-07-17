# TRON2 实机 MVP 联调记录模板

## 0. 环境准备

- `source /home/tron2-loco-gazebo-sim/devel/setup.bash`
- 确认 `policy.onnx` 和 `encoder.onnx` 放在：
  - `tron2_controllers/config/SF_TRON2A/policy/`
  - `tron2_controllers/config/WF_TRON2A/policy/`
- 确认 `robot_ip` 与实机一致（默认 launch 为 `10.192.1.2`）
- 确认遥控器按键映射与 `tron2_hw/config/joystick.yaml` 一致

## 1. 启动命令

### SF_TRON2A

```bash
roslaunch tron2_hw tron2_hw.launch robot_type:=SF_TRON2A robot_ip:=10.192.1.2
```

### WF_TRON2A

```bash
roslaunch tron2_hw tron2_hw.launch robot_type:=WF_TRON2A robot_ip:=10.192.1.2
```

## 2. 控制器与安全接口

- 启动控制器（服务）：

```bash
rosservice call /tron2_hw/start_controller
```

- 停止控制器（服务）：

```bash
rosservice call /tron2_hw/stop_controller
```

- 急停（服务）：

```bash
rosservice call /tron2_hw/emergency_stop
```

- 清除急停（标定状态满足时）：

```bash
rosservice call /tron2_hw/reset_emergency_stop
```

- 遥控快捷键：
  - `L1 + Y`：启动控制器
  - `L1 + X`：急停并锁存

## 3. MVP 分阶段验收

### 阶段 A：单关节读写回环（离地/工装）

- [ ] `joint_states` 连续发布且关节名称顺序符合 `Tron2Cfg.init_state.joint_names`
- [ ] IMU 句柄可用（`limx_imu`），姿态/角速度/加速度数值正常
- [ ] 设置低增益后，单关节指令方向与幅值正确

### 阶段 B：全关节空载低增益

- [ ] 所有关节无异常振荡/打颤
- [ ] 停止控制器后命令能回落到阻尼状态
- [ ] 急停服务与遥控急停立即生效

### 阶段 C：站立与低速运动

- [ ] 30 秒内稳定进入控制状态
- [ ] `/cmd_vel` 指令可驱动机器人响应
- [ ] 低速直行、转向可控，无持续异常抖振

### 阶段 D：SF/WF 双机型复核

- [ ] SF_TRON2A 完成 A/B/C
- [ ] WF_TRON2A 完成 A/B/C
- [ ] 两机型均可重复启动与停控

## 4. 问题记录

| 日期 | 机型 | 现象 | 复现步骤 | 处理结果 |
|---|---|---|---|---|
| YYYY-MM-DD | SF/WF | - | - | - |
