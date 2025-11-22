# damiao_motor_driver 使用说明

`damiao_motor_driver` 是达妙电机控制系统的 ROS 驱动包，提供完整的硬件接口、ros_control 集成和 MoveIt 支持，支持最多 30 路电机的实时控制。

## 目录

- [架构概述](#架构概述)
- [环境依赖](#环境依赖)
- [编译安装](#编译安装)
- [硬件连接](#硬件连接)
- [快速开始](#快速开始)
- [核心功能](#核心功能)
- [MoveIt 集成](#moveit-集成)
- [配置说明](#配置说明)
- [测试与诊断](#测试与诊断)
- [故障排查](#故障排查)

## 架构概述

`damiao_motor_driver` 采用分层架构设计：

- **核心层** (`damiao_motor_core`): 纯 C++ 库，无 ROS 依赖，提供底层通信和驱动功能
- **ROS 包装层** (`damiao_motor_driver`): ROS 接口封装，提供话题、服务、ros_control 集成
- **应用层**: 高级应用功能（RL 策略推理、数据记录等）

### 主要节点

- **`motor_driver_node`**: 核心驱动节点，独占串口，提供 ROS 服务和话题接口
- **`motor_hw_interface_node`**: ros_control 硬件接口节点，用于 MoveIt 集成
- **`motor_cmd_bridge_node`**: 话题桥接节点，将话题命令转发到服务接口

### 主要 Launch 文件

| Launch 文件 | 功能说明 |
|------------|---------|
| `motor_driver.launch` | 启动基础驱动节点，提供服务和话题接口 |
| `motor_cmd_bridge.launch` | 启动话题桥接节点，支持通过话题控制电机 |
| `motor_hw_interface.launch` | 启动硬件接口节点（ros_control） |
| `motor_hw_with_moveit.launch` | 启动硬件接口和控制器，用于 MoveIt 集成 |
| `motor_hw_with_effort_controller.launch` | 启动硬件接口和力矩控制器 |
| `moveit_with_motor_driver.launch` | 一键启动硬件接口、MoveIt 和控制器 |
| `motor_hw_with_rl.launch` | 启动硬件接口和强化学习策略桥接 |

## 环境依赖

### 系统要求

- Ubuntu 20.04 + ROS Noetic Desktop-Full
- 基础工具：`catkin_tools` 或 `catkin_make`、`rosdep`
- Python 依赖（用于策略桥接/ONNX 推理）：`onnxruntime`、`torch`（按需安装）

### 依赖包

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 硬件要求

- 达妙电机控制板（串口，默认 `/dev/mcu` 或 `/dev/ttyUSB0`）
- USB3 线缆（推荐，用于高速率通信）
- 线长 < 1.5m（保证通信稳定性）

## 编译安装

```bash
cd /path/to/ludan_ws_zhengbi
catkin_make  # 或 catkin build
source devel/setup.bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
```

## 硬件连接

### 串口权限设置

1. 连接控制板并确认设备节点：
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB* /dev/mcu
   ```

2. 设置权限（或配置 udev 规则）：
   ```bash
   sudo chmod a+rw /dev/mcu
   ```

3. 如需永久设置，可创建 udev 规则：
   ```bash
   sudo nano /etc/udev/rules.d/99-damiao-motor.rules
   ```
   添加内容：
   ```
   KERNEL=="ttyACM*", ATTRS{idVendor}=="xxxx", MODE="0666"
   ```

## 快速开始

### 1. 基础驱动测试

启动核心驱动节点，验证串口通信和电机反馈：

```bash
roslaunch damiao_motor_driver motor_driver.launch port:=/dev/mcu baud:=921600
```

**参数说明：**
- `port`: 串口设备路径，默认 `/dev/mcu`
- `baud`: 波特率，默认 `921600`
- `ns`: 命名空间，默认 `motor_driver`
- `tf_prefix`: TF 前缀，用于多设备场景

**查看反馈：**
```bash
# 查看电机状态
rostopic echo /motor_driver/motor_states

# 查看诊断信息
rostopic echo /motor_driver/status

# 使用 rqt_plot 可视化
rqt_plot /motor_driver/motor_states/state[0]/pos
```

### 2. 通过服务接口控制电机

**单个电机命令：**
```bash
rosservice call /motor_driver/motor_driver/send_command \
  "{id: 0, p: 0.3, v: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}"
```

**批量电机命令：**
```bash
rosservice call /motor_driver/motor_driver/send_commands \
  "{commands: [{id: 0, p: 0.3, v: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}, \
               {id: 1, p: 0.5, v: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}]}"
```

**参数说明：**
- `id`: 电机 ID（0-29）
- `p`: 目标位置（弧度）
- `v`: 目标速度（弧度/秒）
- `kp`: 位置增益
- `kd`: 速度增益
- `torque`: 力矩（Nm）

### 3. 通过话题控制电机

启动话题桥接节点：

```bash
# 首先启动核心驱动节点
roslaunch damiao_motor_driver motor_driver.launch port:=/dev/mcu baud:=921600

# 然后启动话题桥接节点
roslaunch damiao_motor_driver motor_cmd_bridge.launch
```

通过话题发布命令：

```bash
rostopic pub /motor_driver/motor_cmd damiao_motor_driver/MotorCommand \
  "{id: 0, p: 0.3, v: 0.0, kp: 20.0, kd: 1.0, torque: 0.0}" -r 200
```

## 核心功能

### ROS 接口

#### 话题

- **`motor_states`** (`damiao_motor_control_board_serial/MotorStates`)
  - 包含 30 路电机的 `pos/vel/tor` 反馈
  - 发布频率：500 Hz（可配置）

- **`status`** (`diagnostic_msgs/DiagnosticArray`)
  - 串口连接状态、丢包率、重连次数等健康度信息

#### 服务

- **`send_command`** (`damiao_motor_driver/SendCommand`)
  - 发送单个电机命令

- **`send_commands`** (`damiao_motor_driver/SendCommands`)
  - 批量发送电机命令

### ros_control 硬件接口

`MotorHWInterface` 提供标准的 ros_control 硬件接口，支持：

- **位置接口** (`PositionJointInterface`)
- **速度接口** (`VelocityJointInterface`)
- **力矩接口** (`EffortJointInterface`)
- **状态接口** (`JointStateInterface`)

**启动硬件接口：**

```bash
roslaunch damiao_motor_driver motor_hw_with_effort_controller.launch \
  port:=/dev/mcu baud:=921600 loop_hz:=500
```

**查看控制器状态：**
```bash
rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers
```

**通过控制器话题控制：**
```bash
rostopic pub /motor_hw_interface/joint_group_effort_controller/command \
  std_msgs/Float64MultiArray "data: [0.3, 0, 0, ...]"
```

## MoveIt 集成

### 快速启动（推荐）

使用集成启动文件一键启动硬件接口、MoveIt 和控制器：

```bash
roslaunch damiao_motor_driver moveit_with_motor_driver.launch \
  port:=/dev/mcu baud:=921600 \
  joint_mapping:=$(rospack find damiao_motor_driver)/config/joint_mapping_example.yaml
```

该启动文件会自动：
- 启动硬件接口节点
- 加载控制器配置
- 启动 MoveIt 规划组
- 自动启动指定的控制器（如左腿控制器）

### 分步启动

#### 1. 启动硬件接口和控制器

```bash
roslaunch damiao_motor_driver motor_hw_with_moveit.launch \
  port:=/dev/mcu baud:=921600 \
  loop_hz:=500 \
  joint_mapping:=$(rospack find damiao_motor_driver)/config/joint_mapping_example.yaml
```

#### 2. 启动 MoveIt

```bash
roslaunch ludan_moveit_config move_group.launch \
  moveit_controller_manager:=ros_control
```

#### 3. 启动 RViz（可选）

```bash
roslaunch ludan_moveit_config moveit_rviz.launch
```

### 关节名称映射配置

MoveIt 使用关节名称（如 `LeftShoulderPitch`），而电机驱动使用数字 ID（0-29）。需要配置映射关系：

**创建关节映射文件**（参考 `config/joint_mapping_example.yaml`）：

```yaml
joints:
  - name: LeftHipPitch
    id: 18
    kp: 20.0
    kd: 1.0
  - name: LeftHipRoll
    id: 19
    kp: 20.0
    kd: 1.0
  # ... 更多关节映射
```

**在启动文件中加载映射：**

映射文件会在 `motor_hw_with_moveit.launch` 中通过 `joint_mapping` 参数自动加载。

### MoveIt 控制器配置

确保 MoveIt 配置中的 `ros_controllers.yaml` 包含正确的控制器配置：

```yaml
controller_manager_ns: /motor_hw_interface/hardware

left_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - LeftShoulderPitch
    - LeftShoulderRoll
    # ... 更多关节
```

**重要参数：**
- `controller_manager_ns`: 必须指向 `/motor_hw_interface/hardware`
- `type`: 必须使用 `JointTrajectoryController`（提供 `follow_joint_trajectory` action）

### 使用 MoveIt 进行规划和控制

#### 方法一：在 RViz 中交互式控制（推荐）

1. **启动 RViz（如果未自动启动）：**
   ```bash
   roslaunch ludan_moveit_config moveit_rviz.launch
   ```

2. **在 RViz 中操作：**
   - 在 Motion Planning 插件中选择规划组（如 `left_leg`）
   - 使用交互式标记（Interactive Marker）拖动机器人到目标位置
   - 点击 **Plan** 按钮预览轨迹
   - 点击 **Plan & Execute** 执行轨迹到真实硬件

#### 方法二：使用测试脚本（最简单推荐）

**使用提供的测试脚本：**

项目已经提供了一个测试脚本，可以直接使用：

```bash
# 测试左腿控制器（使用默认参数）
rosrun damiao_motor_driver test_moveit_trajectory.py

# 指定控制器名称
rosrun damiao_motor_driver test_moveit_trajectory.py _controller:=left_leg_controller

# 自定义目标位置和执行时间
rosrun damiao_motor_driver test_moveit_trajectory.py \
  _controller:=left_leg_controller \
  _positions:="[0.3, 0.15, 0.0, -0.4, 0.15, 0.0]" \
  _duration:=3.0
```

**脚本参数说明：**
- `~controller`: 控制器名称（默认：`left_leg_controller`）
- `~positions`: 目标位置数组（弧度，默认：`[0.2, 0.1, 0.0, -0.3, 0.1, 0.0]`）
- `~duration`: 执行时间（秒，默认：`2.0`）

**手动创建 Python 脚本（如果需要自定义）：**

如果你想自己编写脚本，可以参考以下示例：

```python
#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def send_trajectory():
    rospy.init_node('test_moveit_trajectory')
    
    # 根据你的控制器名称调整（例如：left_leg_controller）
    controller_name = rospy.get_param('~controller', 'left_leg_controller')
    action_topic = f'/motor_hw_interface/hardware/{controller_name}/follow_joint_trajectory'
    
    # 创建 action 客户端
    client = actionlib.SimpleActionClient(action_topic, FollowJointTrajectoryAction)
    
    rospy.loginfo(f"等待 action 服务器: {action_topic}")
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logerr(f"Action 服务器未就绪: {action_topic}")
        return False
    
    # 创建轨迹目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'LeftHipPitch', 'LeftHipRoll', 'LeftHipYaw',
        'LeftKnee', 'LeftAnklePitch', 'LeftAnkleRoll'
    ]
    
    # 创建轨迹点（从当前位置移动到目标位置）
    point = JointTrajectoryPoint()
    point.positions = [0.2, 0.1, 0.0, -0.3, 0.1, 0.0]  # 目标位置（弧度）
    point.velocities = [0.0] * len(goal.trajectory.joint_names)
    point.time_from_start = rospy.Duration(2.0)  # 2秒到达目标
    
    goal.trajectory.points = [point]
    goal.trajectory.header.stamp = rospy.Time.now()
    
    # 发送目标
    rospy.loginfo("发送轨迹目标...")
    client.send_goal(goal)
    
    # 等待执行完成
    client.wait_for_result()
    
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("轨迹执行成功！")
        return True
    else:
        rospy.logwarn(f"轨迹执行失败，状态: {client.get_state()}")
        return False

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass
```

**运行测试脚本：**
```bash
# 确保已经启动了 moveit_with_motor_driver.launch
chmod +x test_moveit_trajectory.py
rosrun damiao_motor_driver test_moveit_trajectory.py _controller:=left_leg_controller
```

#### 方法三：通过 rostopic pub 直接发布轨迹（快速测试）

**查看可用的控制器和 action 接口：**
```bash
# 查看已启动的控制器
rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers

# 查看 action 接口（例如左腿控制器）
rostopic list | grep follow_joint_trajectory
# 输出示例: /motor_hw_interface/hardware/left_leg_controller/follow_joint_trajectory
```

**使用 rostopic pub 发送轨迹消息（推荐使用 YAML 文件方式）：**

由于命令行中直接使用多行 YAML 格式容易出错，推荐先创建 YAML 文件，然后使用 `-f` 参数加载。

**步骤 1：创建轨迹文件**

创建一个轨迹文件 `test_trajectory.yaml`：

```yaml
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
    frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: 'test_goal'
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names: ['LeftHipPitch', 'LeftHipRoll', 'LeftHipYaw', 'LeftKnee', 'LeftAnklePitch', 'LeftAnkleRoll']
    points:
    - positions: [0.2, 0.1, 0.0, -0.3, 0.1, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: []
      effort: []
      time_from_start:
        secs: 2
        nsecs: 0
```

**步骤 2：发布轨迹**

```bash
rostopic pub /motor_hw_interface/hardware/left_leg_controller/follow_joint_trajectory/goal \
  control_msgs/FollowJointTrajectoryActionGoal -f test_trajectory.yaml
```

**或者使用单行紧凑格式（如果必须使用命令行）：**

```bash
rostopic pub -1 /motor_hw_interface/hardware/left_leg_controller/follow_joint_trajectory/goal \
  control_msgs/FollowJointTrajectoryActionGoal \
  '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ""}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ""}, goal: {trajectory: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ""}, joint_names: ["LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll"], points: [{positions: [0.2, 0.1, 0.0, -0.3, 0.1, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], accelerations: [], effort: [], time_from_start: {secs: 2, nsecs: 0}}]}}}'
```

**注意：** 单行格式虽然可以工作，但可读性差且容易出错，强烈推荐使用 YAML 文件方式。

#### 方法四：查看和监控

**查看控制器状态：**
```bash
# 查看所有控制器
rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers

# 查看关节状态
rostopic echo /motor_hw_interface/hardware/joint_states

# 查看电机反馈
rostopic echo /motor_hw_interface/hardware/motor_states

# 监控 action 状态
rostopic echo /motor_hw_interface/hardware/left_leg_controller/follow_joint_trajectory/feedback
```

**实时可视化：**
```bash
# 使用 rqt_plot 可视化关节位置
rqt_plot /motor_hw_interface/hardware/joint_states/position[0] \
         /motor_hw_interface/hardware/joint_states/position[1] \
         /motor_hw_interface/hardware/joint_states/position[2]
```

#### 快速测试步骤（推荐）

**最简单的测试方法：**

1. **启动系统：**
   ```bash
   roslaunch damiao_motor_driver moveit_with_motor_driver.launch \
     port:=/dev/mcu baud:=921600
   ```

2. **运行测试脚本：**
   ```bash
   # 在新终端中运行
   rosrun damiao_motor_driver test_moveit_trajectory.py
   ```

3. **观察电机运动：**
   ```bash
   # 在另一个终端中查看关节状态
   rostopic echo /motor_hw_interface/hardware/joint_states
   ```

如果看到关节位置变化，说明 MoveIt 成功控制了电机！

#### 测试注意事项

1. **安全第一**：首次测试时，使用较小的位置变化（如 ±0.2 弧度），确保电机不会超出安全范围
2. **检查关节映射**：确保 `joint_mapping_example.yaml` 中的关节名称与控制器配置完全一致
3. **检查控制器状态**：确保控制器已成功启动并处于 `running` 状态
   ```bash
   rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers
   ```
4. **观察反馈**：通过 `rostopic echo` 观察关节状态和电机反馈，确认运动是否按预期执行
5. **如果测试失败**：
   - 检查 action 服务器是否就绪：`rostopic list | grep follow_joint_trajectory`
   - 检查控制器日志：查看启动终端的输出
   - 检查关节名称是否匹配：`rosparam get /motor_hw_interface/hardware/joints`

## 配置说明

### 控制器配置文件

#### `cfg/effort_controllers.yaml`
用于基础力矩控制，包含：
- `joint_state_controller`: 关节状态发布控制器
- `joint_group_effort_controller`: 30 关节力矩控制器

#### `cfg/moveit_controllers.yaml`
用于 MoveIt 集成，包含：
- `joint_state_controller`: 关节状态发布控制器
- `left_arm_controller`: 左臂轨迹控制器
- `right_arm_controller`: 右臂轨迹控制器
- `left_leg_controller`: 左腿轨迹控制器
- `right_leg_controller`: 右腿轨迹控制器
- `neck_controller`: 颈部轨迹控制器
- `upper_body_controller`: 上半身轨迹控制器

### 关节映射配置

`config/joint_mapping_example.yaml` 提供了关节名称到电机 ID 的映射示例。实际使用时需要根据硬件连接修改。

### 动态参数配置

硬件接口支持通过 `rqt_reconfigure` 动态调整参数：

```bash
rqt_reconfigure
```

可调整的参数包括：
- `kp_limit`: 位置增益上限
- `kd_limit`: 速度增益上限
- `torque_limit`: 力矩上限
- `velocity_limit`: 速度上限
- `position_limit`: 位置上限

## 测试与诊断

### 测试节点

#### 单路电机通断测试

```bash
# 首先启动 motor_driver_node
roslaunch damiao_motor_driver motor_driver.launch port:=/dev/mcu baud:=921600

# 然后运行测试节点
rosrun damiao_motor_driver send_single_motor_node _id:=5
```

#### 全链路闭环测试（正弦轨迹）

```bash
# 首先启动 motor_driver_node
roslaunch damiao_motor_driver motor_driver.launch port:=/dev/mcu baud:=921600

# 然后运行测试节点
rosrun damiao_motor_driver trajectory_controller_node
```

节点以 500 Hz 通过 ROS 服务接口发布指令，`pos` 约为 ±0.5rad、`vel` 峰值约 1.57rad/s。

### 诊断脚本

`scripts/` 目录下提供了多个诊断和测试脚本：

- `check_hardware_connection.py`: 检查硬件连接
- `check_registered_joints.py`: 检查注册的关节
- `diagnose_joint_states_issue.py`: 诊断关节状态问题
- `motor_discovery_test.py`: 电机发现测试
- `test_moveit_execution.py`: MoveIt 执行测试

**使用示例：**
```bash
rosrun damiao_motor_driver check_hardware_connection.py
rosrun damiao_motor_driver check_registered_joints.py
```

### 诊断工具

**查看诊断信息：**
```bash
# 使用 rqt_robot_monitor
rqt_robot_monitor

# 或直接查看话题
rostopic echo /motor_driver/status
```

**查看控制器日志：**
```bash
# 检查控制器状态
rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers

# 查看关节状态
rostopic echo /motor_hw_interface/hardware/joint_states
```

## 故障排查

### 常见问题

#### 1. 串口连接失败

**症状：** 节点启动后无法连接串口

**解决方案：**
- 检查串口设备是否存在：`ls /dev/mcu`
- 检查串口权限：`sudo chmod a+rw /dev/mcu`
- 检查串口是否被其他程序占用：`lsof /dev/mcu`
- 尝试不同的串口设备路径

#### 2. MoveIt 找不到控制器

**症状：** MoveIt 提示 "Controller 'xxx' not found"

**解决方案：**
- 确认 `controller_manager_ns` 指向 `/motor_hw_interface/hardware`
- 检查控制器是否已启动：
  ```bash
  rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers
  ```
- 确认控制器配置文件中的关节名称与 MoveIt 配置一致
- 检查控制器类型是否为 `JointTrajectoryController`

#### 3. 关节名称不匹配

**症状：** 关节状态为空或控制器无法找到关节

**解决方案：**
- 确认关节映射文件中的关节名称与 MoveIt 配置完全一致（包括大小写）
- 检查 `joint_mapping` 参数是否正确加载
- 使用 `check_registered_joints.py` 脚本检查注册的关节

#### 4. 执行过程中触发安全模式

**症状：** 电机突然停止，日志提示安全模式

**解决方案：**
- 检查串口连接和供电是否稳定
- 使用 `rqt_reconfigure` 调整 `kp_limit/kd_limit/torque_limit` 等参数
- 检查指令是否超过安全阈值
- 查看诊断信息了解具体错误原因

#### 5. 关节状态不更新

**症状：** `joint_states` 话题没有数据或数据不更新

**解决方案：**
- 确认 `joint_state_controller` 已启动
- 检查硬件接口节点是否正常运行
- 确认串口通信正常（查看 `motor_states` 话题）
- 使用 `check_joint_states.py` 脚本诊断

### 安全停机

**紧急停机：**
```bash
# 调用安全停机服务
rosservice call /motor_hw_interface/safe_stop
```

**正常退出：**
使用 `Ctrl+C` 正常退出节点时，驱动会自动调用 `send_safe_mode_frame`，向所有已知 ID 下发零速度零力矩指令，确保电机安全停车。

### 日志查看

**查看节点日志：**
```bash
# 查看 rosout
rostopic echo /rosout

# 查看特定节点的日志
rosnode info /motor_driver/motor_driver
```

**查看启动日志：**
```bash
# 使用提供的脚本
bash scripts/check_startup_logs.sh
bash scripts/view_startup_logs.sh
```

## 高级功能

### 强化学习策略桥接

`motor_hw_with_rl.launch` 提供了强化学习策略桥接功能，支持将训练好的策略部署到真实硬件。

**启动方式：**
```bash
roslaunch damiao_motor_driver motor_hw_with_rl.launch \
  port:=/dev/mcu \
  policy_path:=/path/to/policy.onnx \
  joint_order:=[0,1,2,3]
```

详细配置请参考相关文档。

### 多设备支持

通过命名空间和 TF 前缀支持多套设备：

```bash
# 第一套设备
roslaunch damiao_motor_driver motor_driver.launch \
  ns:=motor_driver_1 \
  tf_prefix:=robot1 \
  port:=/dev/mcu1 \
  baud:=921600

# 第二套设备
roslaunch damiao_motor_driver motor_driver.launch \
  ns:=motor_driver_2 \
  tf_prefix:=robot2 \
  port:=/dev/mcu2 \
  baud:=921600
```

## 维护与开发

### 代码结构

```
damiao_motor_driver/
├── cfg/                    # 动态参数配置
│   ├── driver_limits.cfg
│   ├── effort_controllers.yaml
│   └── moveit_controllers.yaml
├── config/                 # 静态配置文件
│   ├── controllers.yaml
│   └── joint_mapping_example.yaml
├── include/                # 头文件
│   └── damiao_motor_driver/
├── launch/                 # Launch 文件
├── scripts/                # Python 脚本和工具
├── src/                    # 源代码
├── msg/                    # 消息定义
├── srv/                    # 服务定义
└── test/                   # 测试文件
```

### 扩展开发

- **添加新控制器：** 在 `cfg/moveit_controllers.yaml` 中添加控制器配置
- **修改关节映射：** 编辑 `config/joint_mapping_example.yaml` 或创建新的映射文件
- **添加新功能：** 参考现有节点实现，遵循 ros_control 接口规范

## 许可证

MIT License

## 维护者

Zhengbi Yong (zhengbi.yong@outlook.com)

## 相关资源

- 项目工作空间 README: `src/README.md`
- 底层串口通信包: `damiao_motor_control_board_serial`
- 核心驱动库: `damiao_motor_core`
