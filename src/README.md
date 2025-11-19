# Ludan ROS1 Noetic Workspace

该工作空间整合了达妙电机串口驱动、`ros_control` 硬件接口、强化学习策略桥接等组件，可直接通过 MoveIt! 规划并执行真实硬件的轨迹。本 README 重点说明整体目录结构、依赖、编译方式，以及“驱动 → 控制器 → MoveIt” 的完整启动流程。

## 目录结构

| 目录 | 说明 |
| --- | --- |
| `damiao_motor_control_board_serial` | 与控制板串口通讯的底层封装，负责帧格式解析与串口健康监控，可单独用于调试电机状态。 |
| `damiao_motor_driver` | 面向 ROS 的高层驱动：发布 `motor_states`、提供 `MotorHWInterface`（`ros_control` 插件）、带有示例控制器/策略桥接脚本及多种 launch 文件。 |

除此之外，`cfg/` 与 `config/` 保存控制器参数、动态限幅配置等，`scripts/` 提供强化学习策略桥接工具。

## 环境依赖

- Ubuntu 20.04 + ROS Noetic Desktop-Full。
- 基础工具：`catkin_tools` 或 `catkin_make`、`rosdep`、`python3-roslaunch`。
- Python 依赖（用于策略桥接/ONNX 推理）：`onnxruntime`、`torch`（按需安装）。
- 硬件：达妙电机控制板（串口，默认 `/dev/ttyUSB0`/`/dev/ttyACM0`）。

首次克隆后执行：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 编译工作空间

```bash
cd /path/to/ludan_workspace_ros1_noetic
catkin_make      # 或 catkin build
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```

## 硬件连接与串口权限

1. 连接控制板并确认设备节点，如 `ls /dev/ttyACM*`。
2. 设置权限（或配置 udev 规则）：
   ```bash
   sudo chmod a+rw /dev/ttyACM0
   ```
3. 若需要高速率，请使用 USB3 线缆，并保证线长 <1.5m。

## 基础驱动测试

### 直接启动 `motor_driver`

用于验证 30 路电机反馈：
```bash
roslaunch damiao_motor_driver motor_driver.launch port:=/dev/ttyACM0 baud:=921600
```
该节点会发布 `motor_states` 与 `status`，可使用 `rostopic echo` 或 `rqt_plot` 观察。【F:src/damiao_motor_driver/launch/motor_driver.launch†L1-L13】【F:src/damiao_motor_driver/src/motor_driver.cpp†L14-L86】

### 串口板级调试

如果只需检查控制板与若干 ID 是否在线，可运行：
```bash
roslaunch damiao_motor_control_board_serial test_motor.launch motor_ids:="[4,5,6]"
```
Launch 文件会加载 `test_motor` 节点并持续打印反馈，有助于确认线路与电源。【F:src/damiao_motor_control_board_serial/launch/test_motor.launch†L1-L12】

## MotorHWInterface 与 ros_control

`damiao_motor_driver` 内置 `MotorHWInterface`，能将 30 路电机映射为 `joint_0 ... joint_29` 的位置/速度/力矩接口，并通过动态重构限制 KP/KD/力矩/速度/位置指令范围。【F:src/damiao_motor_driver/include/damiao_motor_driver/motor_hw_interface.h†L1-L55】【F:src/damiao_motor_driver/cfg/driver_limits.cfg†L1-L13】

### 启动硬件接口 + 控制器

推荐使用 `motor_hw_with_effort_controller.launch`：
```bash
roslaunch damiao_motor_driver motor_hw_with_effort_controller.launch \
  port:=/dev/ttyACM0 baud:=921600 loop_hz:=500
```
该 Launch 做了三件事：
1. 启动 `motor_hw_interface_node`，内部加载 `MotorHWInterface` 插件并提供 `safe_stop` 服务、看门狗诊断。【F:src/damiao_motor_driver/launch/motor_hw_with_effort_controller.launch†L1-L24】【F:src/damiao_motor_driver/src/motor_hw_interface_node.cpp†L1-L106】
2. 加载 `cfg/effort_controllers.yaml`，默认包含 `joint_state_controller` 与 `joint_group_effort_controller`（30 关节）。【F:src/damiao_motor_driver/launch/motor_hw_with_effort_controller.launch†L4-L20】【F:src/damiao_motor_driver/cfg/effort_controllers.yaml†L1-L33】
3. 通过 `controller_manager/spawner` 启动上述控制器，`joint_group_effort_controller/command` 话题即为对电机的力矩接口。【F:src/damiao_motor_driver/launch/motor_hw_with_effort_controller.launch†L21-L25】

启动成功后，可使用：
```bash
rosservice call /motor_hw_interface/controller_manager/list_controllers
rostopic echo /motor_hw_interface/motor_states
``` 
确认控制器运行与反馈正常。

### 内建测试节点

- `rosrun damiao_motor_driver trajectory_controller_node`：让所有电机按照正弦轨迹运动，适合快速自检。【F:src/damiao_motor_driver/src/trajectory_controller_node.cpp†L1-L200】
- `rosrun damiao_motor_driver send_single_motor_node _id:=5`：指定 ID 的单路通断测试。【F:src/damiao_motor_driver/src/send_single_motor_node.cpp†L1-L160】

## 通过 MoveIt! 启动与控制

以下流程假设你已经有对应机械臂的 `moveit_config` 包（例如 `my_robot_moveit_config`），其中使用 `ros_control` 作为控制器管理后端。

### 操作总览

1. **驱动/硬件接口**：启动 `motor_hw_interface_node` 并加载力矩控制器，向外提供标准的 `follow_joint_trajectory` action。【F:src/damiao_motor_driver/launch/motor_hw_with_effort_controller.launch†L1-L25】
2. **MoveIt 控制器配置**：在 `moveit_config` 中声明 `joint_group_effort_controller`，确保 `move_group` 能把规划的轨迹投递给 `ros_control`。【F:src/damiao_motor_driver/cfg/effort_controllers.yaml†L1-L33】
3. **MoveIt 规划执行**：启动 `moveit_planning_execution` 与 `moveit_rviz`，在 RViz 中进行规划并执行到真实硬件。

### 详细步骤：使用 MoveIt 驱动达妙电机

1. **准备工作空间与硬件**  
   - 按“环境依赖”“编译工作空间”“硬件连接”章节完成 `catkin_make`、串口授权等准备。  
   - 确认控制板在 `/dev/ttyACM0`（或自定义）可用，并记录波特率。

2. **启动 MotorHWInterface + 控制器**  
   在第一个终端中执行：
   ```bash
   roslaunch damiao_motor_driver motor_hw_with_effort_controller.launch \
     port:=/dev/ttyACM0 baud:=921600 loop_hz:=500
   ```
   该 launch 会：
   - 加载 `MotorHWInterface` 插件，并在 `/motor_hw_interface` 命名空间下发布 `joint_states`、`motor_states` 以及 `follow_joint_trajectory` action。  
   - 通过 `controller_manager` 启动 `joint_state_controller` 与 `joint_group_effort_controller`，`/motor_hw_interface/follow_joint_trajectory` 即是 MoveIt 将要连接的 action 服务器。【F:src/damiao_motor_driver/launch/motor_hw_with_effort_controller.launch†L1-L25】
   - 可使用 `rosservice call /motor_hw_interface/controller_manager/list_controllers` 验证控制器状态为 `running`。【F:src/damiao_motor_driver/src/motor_hw_interface_node.cpp†L33-L106】

3. **配置 MoveIt 控制器**  
   在你的 `my_robot_moveit_config/config/ros_controllers.yaml`（或等效文件）中添加/确认以下内容：
   ```yaml
   controller_manager_ns: /motor_hw_interface
   controller_list:
     - name: joint_group_effort_controller
       action_ns: follow_joint_trajectory
       type: FollowJointTrajectory
       default: true
       joints:
         - joint_0
         - joint_1
         # ... 按照机械臂实际使用的关节顺序填写
   ```
   同时确保 MoveIt 的 `moveit_controller_manager.launch.xml` 或 `move_group.launch` 将 `ros_controllers.yaml` 作为参数传入，使 `move_group` 知道去 `/motor_hw_interface` 命名空间下寻找 action。若 URDF/MoveIt 使用自定义关节名，可在 `motor_hw_interface.launch` 中设置 `joint_prefix/tf_prefix` 以保持一致。【F:src/damiao_motor_driver/config/controllers.yaml†L1-L9】

4. **启动 MoveIt 并连接 RViz**  
   在第二个终端中：
   ```bash
   roslaunch my_robot_moveit_config moveit_planning_execution.launch \
     moveit_controller_manager:=ros_control \
     trajectory_execution.allowed_execution_duration_scaling:=1.5
   ```
   若需要可视化/交互规划，再开一个终端运行：
   ```bash
   roslaunch my_robot_moveit_config moveit_rviz.launch
   ```
   在 RViz → MotionPlanning 插件中：
   - 选择机械臂的规划组，点击 **Plan** 预览轨迹。  
   - 点击 **Plan & Execute**，MoveIt 会将轨迹发送到 `/motor_hw_interface/follow_joint_trajectory`，由 `joint_group_effort_controller` 转换成力矩指令并下发给达妙电机。

5. **运行时监控与排障**  
   - 如果 MoveIt 提示“找不到控制器”，请重新检查 `controller_manager_ns` 是否指向 `/motor_hw_interface/controller_manager`，并确认 `joint_group_effort_controller` 已运行。  
   - 若 RViz 规划的关节名与驱动不一致，可通过 `joint_prefix`/`tf_prefix` 参数对齐名称，或在 MoveIt 配置中调整。  
   - 若执行异常或触发安全模式时，可使用 `rqt_reconfigure` 调整 `kp_limit/kd_limit/torque_limit`，或调用 `rosservice call /motor_hw_interface/safe_stop` 立即停机。【F:src/damiao_motor_driver/cfg/driver_limits.cfg†L1-L13】【F:src/damiao_motor_driver/src/motor_hw_interface_node.cpp†L60-L88】

### 常见排障

- **MoveIt 报告找不到控制器**：确认 `controller_manager_ns` 指向 `/motor_hw_interface/controller_manager`，并且 `list_controllers` 中存在 `joint_group_effort_controller` 且状态为 `running`。
- **关节名称不匹配**：`MotorHWInterface` 默认提供 `joint_0...joint_29`，请确保 URDF / MoveIt 配置使用同名关节或在 `joint_prefix` 参数中设置转换（参见 `motor_hw_interface.launch` 中的 `tf_prefix`/`joint_prefix`）。【F:src/damiao_motor_driver/include/damiao_motor_driver/motor_hw_interface.h†L33-L55】
- **执行过程中触发安全模式**：检查串口连接、供电，并通过 `rqt_reconfigure` 调整 `kp_limit/kd_limit/torque_limit` 等参数，避免指令超过阈值。【F:src/damiao_motor_driver/src/motor_driver.cpp†L40-L117】【F:src/damiao_motor_driver/cfg/driver_limits.cfg†L1-L13】

## 强化学习策略桥接

`scripts/policy_bridge.py` 可将 TorchScript/ONNX 模型或外部话题映射到 `joint_group_effort_controller/command`，示例启动文件为 `motor_hw_with_rl.launch`：
```bash
roslaunch damiao_motor_driver motor_hw_with_rl.launch \
  port:=/dev/ttyACM0 policy_path:=/path/to/policy.onnx joint_order:="[0,1,2,3]"
```
该 launch 在硬件接口与控制器之外额外启动策略桥接节点，并将动作发送到 `joint_group_effort_controller/command`。可通过参数调整推理频率、动作裁剪、观测字段等。【F:src/damiao_motor_driver/launch/motor_hw_with_rl.launch†L1-L38】

## 诊断与安全停机

- `motor_hw_interface_node` 内部含有看门狗，若反馈/指令超时会调用 `sendSafeMode` 并通过 `diagnostics` 发布警告。可在 RViz Diagnostics 或 `rqt_robot_monitor` 中查看。【F:src/damiao_motor_driver/src/motor_hw_interface_node.cpp†L33-L97】
- 随时可调用：
  ```bash
  rosservice call /motor_hw_interface/safe_stop
  ```
  让驱动下发零速度/零力矩指令并关闭串口，确保机械臂安全停车。【F:src/damiao_motor_driver/src/motor_hw_interface_node.cpp†L60-L88】

## 提交 / 维护建议

- 修改 launch 或控制器配置后，记得更新对应的 MoveIt `ros_controllers.yaml`，保持关节名称、命名空间一致。
- 若需要扩展电机数量，可在 `config/controllers.yaml` 与 `cfg/effort_controllers.yaml` 中增删关节，并确保硬件接口参数同步。【F:src/damiao_motor_driver/config/controllers.yaml†L1-L9】【F:src/damiao_motor_driver/cfg/effort_controllers.yaml†L1-L33】
- 建议使用 `screen` 或 `tmux` 运行驱动与 MoveIt，以便在远程部署时保持日志。

通过以上流程即可实现从硬件驱动到 MoveIt 规划执行、再到策略桥接/调试的完整闭环。祝使用顺利！
