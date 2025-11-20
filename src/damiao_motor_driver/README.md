# damiao_motor_driver 使用说明

本节说明在代码已成功编译后，如何在实际硬件上测试达妙电机驱动。默认假设已完成 `source devel/setup.bash` 并能访问串口设备。

## 启动驱动节点
1. 确认 USB/串口已经赋予读写权限（例如 `sudo chmod a+rw /dev/ttyACM0`）。
2. 根据实际串口名和波特率配置参数后启动驱动：
   ```bash
   roslaunch damiao_motor_driver motor_driver.launch port:=/dev/ttyACM0 baud:=921600
   ```
   - `port`：与电机控制板相连的串口设备，默认 `/dev/ttyACM0`。
   - `baud`：串口波特率，默认 `921600`。
   - 如有多套设备，可通过 `ns` 与 `tf_prefix` 参数在不同命名空间下重复启动。

驱动启动后会自动通过 `controller_manager` 加载 `joint_state_controller` 与 `joint_group_effort_controller`。因此除了反馈话题，还会额外暴露以下接口：

- `motor_states` (`damiao_motor_control_board_serial/MotorStates`)：包含 30 路电机的 `pos/vel/tor` 反馈。
- `status` (`diagnostic_msgs/DiagnosticArray`)：串口连接状态、丢包率、重连次数等健康度信息。
- `joint_group_effort_controller/command` (`std_msgs/Float64MultiArray`)：力矩控制指令，数组顺序与 `controllers.yaml` 中的 `joints` 列表一致，可通过 `rostopic pub` 或自定义节点写入。

可通过以下命令观测：
```bash
rostopic echo /motor_driver/motor_states
rostopic echo /motor_driver/status
rostopic list | grep joint_group_effort_controller
```

若希望手动测试输出，可执行：
```bash
rostopic pub /motor_driver/joint_group_effort_controller/command std_msgs/Float64MultiArray "data: [0.3, 0, 0, ...]"
```
确保消息长度与启用的关节数量一致即可驱动对应电机。

## 通过 rostopic pub 控制电机
`motor_cmd_bridge_node` 提供了通过 ROS 话题直接控制电机的功能。启动节点后，可以通过 `rostopic pub` 命令或自定义节点向 `/motor_cmd` 话题发布命令来控制电机。

**启动节点：**
```bash
roslaunch damiao_motor_driver motor_cmd_bridge.launch port:=/dev/ttyACM0 baud:=921600
```

**通过命令行控制电机：**
```bash
rostopic pub /motor_cmd damiao_motor_driver/MotorCommand \
  "{id: 0, p: 0.3, v: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}" -r 200
```

参数说明：
- `id`: 电机 ID（0-29）
- `p`: 目标位置（弧度）
- `v`: 目标速度（弧度/秒）
- `kp`: 位置增益
- `kd`: 速度增益
- `torque`: 力矩（Nm）
- `-r 200`: 以 200 Hz 频率持续发布

**通过自定义节点控制：**
其他 ROS 节点可以订阅 `/motor_cmd` 话题并发布 `damiao_motor_driver/MotorCommand` 消息来控制电机。

注意：`motor_cmd_bridge_node` 和 `motor_driver_node` 不能同时运行，因为它们都需要独占串口。如果只需要通过话题控制电机，使用 `motor_cmd_bridge_node` 即可。

## 单路电机通断测试
若仅需验证单个电机收发是否正常，可使用内置节点持续向指定 ID 发送零力矩指令：
```bash
rosrun damiao_motor_driver send_single_motor_node _id:=5
```
- 将 `_id` 换成目标电机的协议 ID。节点启动后会自动打开串口并开始发送命令，按 `Ctrl+C` 可退出；退出时会下发安全帧使电机进入安全模式。

## 全链路闭环测试（sin 轨迹）
要快速验证 30 路电机的控制回路，可运行轨迹控制节点，让所有 ID 按正弦轨迹运动：
```bash
rosrun damiao_motor_driver trajectory_controller_node
```
节点以 500 Hz 发布指令，`pos` 约为 ±0.5rad、`vel` 峰值约 1.57rad/s。运行过程中可结合 `rqt_plot` 观察 `motor_states` 中的 `pos`、`vel` 是否跟随。

## 与 ros_control 集成测试
该包提供了 `hardware_interface::RobotHW` 插件（`motor_hw_interface_plugin.xml` 中的 `damiao_motor_driver/MotorHWInterface`），可在自定义的 `controller_manager` 节点中加载。

典型流程：
1. 在自己的 `controller_manager` 节点中创建 `pluginlib::ClassLoader<hardware_interface::RobotHW>`，加载 `damiao_motor_driver/MotorHWInterface`。插件会自动打开串口、注册位置/速度/力矩/状态接口并启动反馈线程。
2. 使用 `rqt_reconfigure` 调整 `kp_limit/kd_limit/torque_limit/velocity_limit/position_limit`，验证限幅是否生效；当指令超限或串口异常时，驱动会在日志中提示并将指令钳制到安全范围。
3. 通过 `controller_manager` 加载自己的控制器并下发控制命令，观察 `motor_states` 反馈与 `status` 诊断信息，以确认闭环控制链路正常。

## 诊断与安全停机
- 驱动在循环内检测校验和错误、超时和解析失败，累计到阈值会自动重连串口并在 `status` 中标记。可用 `rqt_robot_monitor` 实时查看。
- 正常退出 (`Ctrl+C`) 或检测到严重错误时，节点会调用 `send_safe_mode_frame`，向所有已知 ID 下发零速度零力矩指令，确保电机安全停车。

按照上述流程，即可在硬件上验证驱动的通讯、反馈和控制功能是否正常。

## 策略桥接与离线回放
`scripts/policy_bridge.py` 提供了一个基于 Python/rospy 的轻量级策略桥接节点，便于把上层学习策略或离线控制轨迹映射到电机控制器：

- 订阅 `motor_states` 并按照 `~joint_order` 与 `~observation_fields` 构造观测向量，可选归一化/裁剪（`~observation_scale`、`~observation_clip`）。
- 从 TorchScript/ONNX 权重（`~policy_path`、`~policy_type`）推理，或订阅外部动作话题（`~action_topic`）。动作支持缩放/限幅（`~action_scale`、`~action_clip`）以及低通/阻尼滤波（`~action_filter_alpha`、`~action_damping`）。
- 输出可映射到 `cmd_vel`（Twist）或 `joint_group_effort_controller/command` 等控制器话题。
- 支持 rosbag/CSV 回放（`~playback_source`、`~playback_topic`），并可将反馈记录到 bag/CSV（`~record_bag_path`、`~record_csv_path`）以评估延迟与安全钳位效果。

例如：
```bash
rosrun damiao_motor_driver policy_bridge.py \
  _joint_order:=[0,1,2,3] _policy_path:=/tmp/policy.onnx _command_topic:=/joint_group_effort_controller/command \
  _action_clip:=2.5 _action_filter_alpha:=0.2 _record_bag_path:=/tmp/motor_states.bag
```

## IsaacLab 部署流程
以下步骤可将训练好的策略通过 `policy_bridge` 部署到真实电机：

1. **导出模型**：
   - TorchScript：在训练代码中使用 `torch.jit.trace`/`script` 导出，并将 `.pt` 文件路径传给 `~policy_path`，`~policy_type` 设为 `torch`（或 `auto` 让脚本自动识别）。
   - ONNX：使用 `torch.onnx.export` 导出 `.onnx`，部署时安装 `onnxruntime`（`pip install onnxruntime`），将 `~policy_type` 设为 `onnx`。
2. **启动硬件 + 控制器**：
   - 推荐直接运行示例启动文件：
     ```bash
     roslaunch damiao_motor_driver motor_hw_with_rl.launch policy_path:=/path/to/policy.onnx joint_order:=[0,1,2,3]
     ```
   - 该启动文件会在命名空间 `motor_hw_interface` 下加载 `motor_hw_interface_node`、`joint_state_controller`、`joint_group_effort_controller`，并把策略输出映射到 `joint_group_effort_controller/command`。
3. **桥接参数检查**：
   - `~joint_order` 必须与 IsaacLab 训练时的关节索引一致；`~observation_fields` 默认 `[pos, vel, tor]`，可根据训练观测调整。
   - `~action_clip`/`~action_scale` 控制力矩缩放与限幅，避免策略输出超出驱动器安全范围；`~action_filter_alpha`/`~action_damping` 可平滑高频抖动。
   - `~motor_states_topic`、`~command_topic` 可匹配其他命名空间或控制器名称。
4. **常见故障排查**：
   - **策略加载失败**：确认 `policy_path` 存在且与 `policy_type` 匹配；若是 ONNX，确保系统已安装 `onnxruntime`。
   - **无力矩输出/动作全为零**：检查 `action_clip` 是否过小，或 `policy_bridge` 日志是否提示动作被限幅；同时确认控制器已被 spawner 正常加载（`rosservice call /motor_hw_interface/controller_manager/list_controllers`）。
   - **观测维度不匹配**：当终端报错 "parameter with length..." 时，核对 `joint_order` 长度是否与动作/观测维度一致，必要时在 IsaacLab 侧重新导出策略。
