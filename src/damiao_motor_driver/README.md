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

驱动启动后将周期性发布两类话题：
- `motor_states` (`damiao_motor_control_board_serial/MotorStates`)：包含 30 路电机的 `pos/vel/tor` 反馈。
- `status` (`diagnostic_msgs/DiagnosticArray`)：串口连接状态、丢包率、重连次数等健康度信息。

可通过以下命令观测：
```bash
rostopic echo /motor_driver/motor_states
rostopic echo /motor_driver/status
```

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
