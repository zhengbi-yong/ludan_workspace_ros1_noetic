版本10.15：
更新了moveit，可以使用moveit控制四肢
版本 09.24:
这个版本可以实现一套ros1 noetic控制多个mcu 目前是两个
启动的时候启动multi_bringup.launch就可以了
测试流程：


# 灵巧手1
(1)启动
roslaunch inspire_hand multi_bringup.launch

(2)
rosservice call /lefthand/inspire_hand/set_angle 500 500 500 500 500 500
rosservice call /lefthand/inspire_hand/set_angle 1000 1000 1000 1000 1000 1000
rosservice call /righthand/inspire_hand/set_angle 500 500 500 500 500 500
rosservice call /righthand/inspire_hand/set_angle 1000 1000 1000 1000 1000 1000
rosservice call /lefthand/inspire_hand/set_angle angle1 angle2 angle3 angle4 angle5 angle6 
rosservice call /righthand/inspire_hand/set_angle angle1 angle2 angle3 angle4 angle5 angle6
设置灵巧手角度------参数angle范围-1-1000

(3)
rosservice call /lefthand/inspire_hand/set_force force1 force2 force3 force4 force5 force6 
rosservice call /righthand/inspire_hand/set_force force1 force2 force3 force4 force5 force6 
设置力控阈值------参数force范围0-1000

(4)
rosservice call /lefthand/inspire_hand/set_speed speed1 speed2 speed3 speed4 speed5 speed6 
rosservice call /righthand/inspire_hand/set_speed speed1 speed2 speed3 speed4 speed5 speed6 
设置速度------参数speed范围0-1000

(5)
rosservice call /lefthand/inspire_hand/get_pos_act
rosservice call /righthand/inspire_hand/get_pos_act
读取驱动器实际的位置值

(6)
rosservice call /lefthand/inspire_hand/get_angle_act
rosservice call /righthand/inspire_hand/get_angle_act
读取实际的角度值

# 灵巧手2
(7)
监测数据:

roslaunch inspire_hand dual_hand.launch
rostopic echo /lefthand/angle_data
rostopic echo /righthand/angle_data

rostopic echo /lefthand/force_data
rostopic echo /righthand/force_data

rostopic pub -1 /righthand/set_hand_command inspire_hand/hand_command "{
finger_ids: [1, 2, 3, 4, 5, 6],
angles: [1000, 1000, 1000, 1000, 1000, 1000],
forces: [200, 200, 200, 200, 200, 200],
speeds: [800, 800, 800, 800, 800, 800]
}"

#添加 机器人模型
# 1) 路径变量（确认 roscore 已起 & 已 source 工作空间）
URDF="$(rospack find yeah)/urdf/yeah.urdf"

# 2) 生成临时 YAML：用文本块 | 包起来，并缩进两格
{ echo "robot_description: |"; sed 's/^/  /' "$URDF"; } > /tmp/robot_description.yaml

# 3) 加载到参数服务器
rosparam load /tmp/robot_description.yaml

# 4) 自检：应能看到开头几行 XML
rosparam get /robot_description | head -n 5

# 5) 起 TF（没有真实关节源就起一个关节发布器）
rosrun robot_state_publisher robot_state_publisher


# debug
## 监听发的命令是否收到
rostopic echo /mcu_leftarm/all_joints_hjc/command_moveJ 
rostopic echo /mcu_leftarm/all_joints_hjc/command_oneroslaunch ludan_moveit_config move_group.launch
/right_arm_controller/follow_joint_trajectory/goal
## 监听关节角度
rostopic echo /mcu_leftarm/joint_states 



# 1. 硬件驱动启动
当前版本启动多个mcu
roslaunch simple_hybrid_joint_controller multi_bringup.launch
# 2.单电机运动测试
### 左右臂控制
#### id mcu_rightarm 7 8 9 10 11 12 13 
rostopic pub mcu_rightarm/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [13, 1.0, 0.0, 5.0, 1.0, 0.0]"

rostopic pub mcu_rightarm1/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"

rostopic pub mcu_rightarm3/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [7, 8.2, 0.0, 15.0, 1.0, 0.0]"

rostopic pub mcu_rightarm3/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_leftarm 7 8 9 10 11 12 13
rostopic pub mcu_leftarm/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_neck 7 10 9
rostopic pub mcu_rightarm1/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_rightleg 7 8 9 10 11 12
rostopic pub mcu_rightleg/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [7, 0.0, 0.0, 30.0, 1.0, 0.0]"
#### id mcu_leftleg 7 8 9 10 11 12
rostopic pub mcu_leftleg/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [7, 0.0, 0.0, 30.0, 1.0, 0.0]"
# 3.MoveJ运动测试
### MoveJ 
rostopic pub mcu_leftarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

rostopic pub mcu_rightarm3/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [1.5, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0]"
rostopic pub mcu_leftarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.0, 0.0, 0.0, 0.5, 0.5]"
rostopic pub mcu_rightarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rostopic pub mcu_rightarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
rostopic pub mcu_leftleg/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rostopic pub mcu_leftleg/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, -1.3, 0.0, 2.53, 0.48, 0.0]"
rostopic pub mcu_rightleg/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

## 所有电机一起
rostopic pub -r 20 mcu_leftarm/all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [1.0, 0.0, 5.0, 1.0, 0.0]"

rostopic pub -r 20 mcu_rightarm3/all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [1.0, 0.0, 5.0, 1.0, 0.0]"
## 急停
rostopic pub -r 20 mcu_rightleg/all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
## 单个电机运动
### 速度控制 speed control
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 0, 0.2, 0, 1, 1]"10
### 位置控制 pos control
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 1.0, 0.0, 1.0, 1.0, 0.0]"
# 4.Moveit运动测试
### MoveIt
#### 打开硬件驱动
roslaunch simple_hybrid_joint_controller multi_bringup.launch
#### 打开moveit和moveJ的桥梁
roslaunch moveit_movej_bridge bridge.launch
#### 打开moveit
roslaunch ludan_moveit_config move_group.launch
#### 打开姿态发布
rosrun robot_state_publisher robot_state_publisher
#### 打开rviz测试
rosrun rviz rviz -d $(rospack find ludan_moveit_config)/launch/moveit.rviz -f Body_Base
##### rviz测试流程
     add Robotstate
     add MotionPlanning
     Joints set angle
     plan
     execute
##### 简易测试流程
rosrun moveit_commander moveit_commander_cmdline.py 
use right_arm
current
x = [12.665999984741211 0.11178779602050781 0.08525943756103516 0.07038211822509766 0.08907413482666016 -12.5]
plan x
execute



#### 逐个关节测试示例
先按照上面的顺序启动底层、桥接和 MoveIt，再单独启动测试脚本：

```
rosrun moveit_movej_bridge sequential_joint_test.py _group:=right_arm _delta:=0.2 _pause:=2.0
```

参数说明：

* `group`：MoveIt 中的规划组名称，例如 `right_arm`、`left_leg` 等。
* `delta`：单个关节在测试过程中尝试偏转的弧度值，默认为 `0.2`。
* `pause`：每次关节运动后的停顿时间（秒），默认为 `2.0`，便于观察电机状态。

脚本会记录当前关节位置，并逐个关节在安全范围内偏转，帮助检查电机响应是否正常。全部关节测试完成后会回到起始位置。

### 使用全身 URDF 通过 MoveIt 控制右腿
当需要在加载整机 URDF 的情况下验证右腿动作时，可以按照下面的顺序启动节点：

1. **底层驱动**：
   ```bash
   roslaunch simple_hybrid_joint_controller multi_bringup.launch
   ```
   该 launch 文件会一次性启动左右臂、左右腿以及颈部的 MCU 通讯节点，确保完整的机械体都已经连上。默认串口设备为 `/dev/mcu_XXXX`，若设备名不同可通过 `port` 参数覆盖。

2. **MoveIt-桥接层**：
   ```bash
   roslaunch moveit_movej_bridge bridge.launch
   ```
   这个桥接会加载 `joint_remap.yaml` 以及各个子部位的映射文件，负责把 MoveIt 输出的 `FollowJointTrajectory` 目标转换成底层 `command_moveJ` 话题，并合并来自多路 MCU 的 `joint_states`。

3. **加载整机 MoveIt 配置**：
   ```bash
   roslaunch ludan_moveit_config move_group.launch
   ```
   该配置引用整机 URDF（`yeah.srdf` 中包含 `right_leg` 规划组），允许在 MoveIt 中同时看到全部关节。

4. **可视化与交互**（可选）：
   ```bash
   roslaunch ludan_moveit_config moveit_rviz.launch
   ```
   RViz 中加载的默认配置已包含 MotionPlanning 面板。若需要手动启动 RViz，也可以运行 `rosrun rviz rviz -d $(rospack find ludan_moveit_config)/launch/moveit.rviz`。

5. **右腿逐关节验证**：
   ```bash
   rosrun moveit_movej_bridge sequential_joint_test.py _group:=right_leg _delta:=0.2 _pause:=2.0
   ```
   * 如果需要减小运动幅度，可根据实际情况调小 `_delta`。脚本会读取右腿规划组的关节限位，并在安全区间内来回摆动，以确认每个驱动器响应正常。
   * 在 RViz 中，可在 MotionPlanning 面板选择 `right_leg` 作为规划组，使用交互式标定或关节滑块进行额外测试，然后点击 *Plan*/*Execute* 观察硬件反馈。

> **提示**：在整机模式下，确保 `tf` 和 `joint_state` 的时间戳同步。若 RViz 报错，可以降低桥接节点中的 `stream_rate` 或检查 MCU 的串口波特率设置。

# 电池监测
roslaunch jbd_bms_status jbd_bms_status.launch

topic指令
rostopic echo /jbd_bms_status_node/jbd_bms（常用）
rostopic echo /diagnostics

# 其他注意事项
## 1. 打开端口的方法
rosparam set /port /dev/mcu_rightarm
rosparam set /baud 921600
rosparam get /port
rosparam get /baud
sudo chmod a+rw /dev/mcu_rightarm
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
## 2. 绑定串口为指定名字
输入
udevadm info -a -n /dev/ttyACM6 | grep -E "idVendor|idProduct|serial"
udevadm info -a -n /dev/ttyUSB2

输入
ll /etc/udev/rules.d/99-stm32.rules
添加：
ATTRS{serial}=="375539423233"     # [LOG] 你的设备序列号：STM32芯片唯一标识
ATTRS{idProduct}=="5740"          # [LOG] 产品ID：STM32虚拟串口产品代码
ATTRS{idVendor}=="0483"           # [LOG] 厂商ID：STMicroelectronics (STM32制造商)
376A396C3233

重新加载：
   sudo udevadm control --reload-rules
sudo udevadm trigger

检查是否创建成功：
ll /dev/mcu_rightarm

####light
roslaunch test_led pub_node.launch
 rostopic pub /led_mode std_msgs/UInt8MultiArray "data: [1, 255, 255, 0, 100, 1]"

左手ttyUSB2 1-4.3.4:1.0
右手ttyUSB1 1-4.3.1:1.0
