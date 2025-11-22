#!/usr/bin/env python3
"""
测试 MoveIt 轨迹执行脚本
用于验证 MoveIt 是否能正确控制电机运动
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys


def send_trajectory():
    rospy.init_node('test_moveit_trajectory')

    # 根据你的控制器名称调整（例如：left_leg_controller）
    controller_name = rospy.get_param('~controller', 'left_leg_controller')
    action_topic = f'/motor_hw_interface/hardware/{controller_name}/follow_joint_trajectory'

    # 创建 action 客户端
    client = actionlib.SimpleActionClient(
        action_topic, FollowJointTrajectoryAction)

    rospy.loginfo(f"等待 action 服务器: {action_topic}")
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logerr(f"Action 服务器未就绪: {action_topic}")
        rospy.logerr("请确保：")
        rospy.logerr("1. moveit_with_motor_driver.launch 已启动")
        rospy.logerr(f"2. {controller_name} 控制器已启动")
        rospy.logerr(
            f"3. 使用以下命令检查: rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers")
        return False

    # 检查控制器是否正在运行
    rospy.loginfo("检查控制器状态...")
    try:
        from controller_manager_msgs.srv import ListControllers
        list_controllers_service = '/motor_hw_interface/hardware/controller_manager/list_controllers'
        rospy.wait_for_service(list_controllers_service, timeout=2.0)
        list_client = rospy.ServiceProxy(
            list_controllers_service, ListControllers)
        response = list_client()
        controller_found = False
        for controller in response.controller:
            if controller.name == controller_name:
                controller_found = True
                rospy.loginfo(
                    f"控制器 '{controller_name}' 状态: {controller.state}")
                if controller.state != 'running':
                    rospy.logwarn(f"⚠ 控制器 '{controller_name}' 未处于 running 状态！")
                    rospy.logwarn("   这可能导致轨迹执行失败")
                break
        if not controller_found:
            rospy.logwarn(f"⚠ 未找到控制器 '{controller_name}'")
            rospy.logwarn(
                f"   可用的控制器: {[c.name for c in response.controller]}")
    except rospy.ROSException:
        rospy.logwarn("⚠ 无法连接到控制器管理器服务（可能正常，如果控制器管理器未启动）")
    except Exception as e:
        rospy.logwarn(f"⚠ 检查控制器状态时出错: {str(e)}")

    # 获取关节名称（根据控制器类型）
    joint_names_map = {
        'left_leg_controller': ['LeftHipPitch', 'LeftHipRoll', 'LeftHipYaw', 'LeftKnee', 'LeftAnklePitch', 'LeftAnkleRoll'],
        'right_leg_controller': ['RightHipPitch', 'RightHipRoll', 'RightHipYaw', 'RightKnee', 'RightAnklePitch', 'RightAnkleRoll'],
        'left_arm_controller': ['LeftShoulderPitch', 'LeftShoulderRoll', 'LeftShoulderYaw', 'LeftElbowPitch', 'LeftElbowYaw', 'LeftWristPitch', 'LeftWristRoll'],
        'right_arm_controller': ['RightShoulderPitch', 'RightShoulderRoll', 'RightShoulderYaw', 'RightElbowPitch', 'RightElbowYaw', 'RightWristPitch', 'RightWristRoll'],
    }

    joint_names = joint_names_map.get(controller_name, [])
    if not joint_names:
        rospy.logwarn(f"未知的控制器名称: {controller_name}")
        rospy.logwarn("请使用 ~joint_names 参数指定关节名称，例如：")
        rospy.logwarn(
            "  _joint_names:='[\"LeftHipPitch\", \"LeftHipRoll\", ...]'")
        joint_names = rospy.get_param('~joint_names', [])
        if not joint_names:
            rospy.logerr("无法确定关节名称，退出")
            return False

    # 获取目标位置（默认值）
    target_positions = rospy.get_param(
        '~positions', [0.0, 0.2, 0.1, -0.0, 0.0, 0.0])
    # 根据位置变化自动调整执行时间，确保运动平滑
    base_duration = rospy.get_param('~duration', 3.0)  # 默认3秒，更平滑

    if len(target_positions) != len(joint_names):
        rospy.logerr(
            f"目标位置数量 ({len(target_positions)}) 与关节数量 ({len(joint_names)}) 不匹配")
        return False

    # 验证关节名称是否在硬件接口中注册
    rospy.loginfo("检查关节配置...")
    try:
        registered_joints = rospy.get_param(
            '/motor_hw_interface/hardware/joints', [])
        if registered_joints:
            registered_names = []
            if isinstance(registered_joints, list) and len(registered_joints) > 0:
                if isinstance(registered_joints[0], dict):
                    registered_names = [
                        j.get('name', '') for j in registered_joints if isinstance(j, dict)]
                elif isinstance(registered_joints[0], str):
                    registered_names = registered_joints

            rospy.loginfo(f"硬件接口注册的关节: {registered_names}")

            # 检查所有关节是否都已注册
            missing_joints = [
                name for name in joint_names if name not in registered_names]
            if missing_joints:
                rospy.logwarn(f"⚠ 以下关节未在硬件接口中注册: {missing_joints}")
                rospy.logwarn("   这可能导致轨迹执行失败")
                rospy.logwarn(
                    "   请检查 joint_mapping_example.yaml 中的关节名称是否与控制器配置一致")
        else:
            rospy.logwarn("⚠ 无法获取硬件接口的关节配置")
    except Exception as e:
        rospy.logwarn(f"⚠ 检查关节配置时出错: {str(e)}")

    # 获取当前位置
    rospy.loginfo("获取关节当前位置...")
    current_positions = None
    try:
        joint_states_topic = '/motor_hw_interface/hardware/joint_states'
        rospy.loginfo(f"等待关节状态消息: {joint_states_topic}")
        joint_state_msg = rospy.wait_for_message(
            joint_states_topic, JointState, timeout=5.0)

        # 创建关节名称到索引的映射
        joint_name_to_index = {name: i for i,
                               name in enumerate(joint_state_msg.name)}

        # 提取当前位置
        current_positions = []
        for joint_name in joint_names:
            if joint_name in joint_name_to_index:
                idx = joint_name_to_index[joint_name]
                current_positions.append(joint_state_msg.position[idx])
            else:
                rospy.logwarn(
                    f"⚠ 关节 '{joint_name}' 不在 joint_states 中，使用目标位置作为起始位置")
                # 如果找不到，使用目标位置（假设已经在目标位置）
                current_positions.append(
                    target_positions[len(current_positions)])

        rospy.loginfo(f"当前位置: {[f'{p:.3f}' for p in current_positions]}")
    except rospy.ROSException:
        rospy.logwarn("⚠ 无法获取当前位置，将使用目标位置作为起始位置")
        current_positions = target_positions.copy()
    except Exception as e:
        rospy.logwarn(f"⚠ 获取当前位置时出错: {str(e)}，使用目标位置作为起始位置")
        current_positions = target_positions.copy()

    # 创建轨迹目标
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    # 设置路径容差和目标容差（允许一定的误差）
    goal.path_tolerance = []
    goal.goal_tolerance = []
    goal.goal_time_tolerance = rospy.Duration(1.0)  # 允许1秒的时间误差

    # 计算最大位置变化，用于设置合理的容差
    max_position_change = max(
        [abs(t - c) for t, c in zip(target_positions, current_positions)])

    for joint_name in joint_names:
        from control_msgs.msg import JointTolerance
        # 路径容差：轨迹执行过程中的允许误差
        # 设置为最大位置变化的 3 倍，确保不会因为路径跟踪误差而失败
        # 对于大范围运动，需要更大的路径容差
        path_tol = JointTolerance()
        path_tol.name = joint_name
        # 至少 0.3 弧度，或最大变化的 3 倍
        path_tol.position = max(0.3, max_position_change * 3.0)
        path_tol.velocity = 2.0  # 2.0 rad/s 容差（增大以允许更大的速度误差）
        goal.path_tolerance.append(path_tol)

        # 目标容差：到达目标时的允许误差
        # 设置为最大位置变化的 1.5 倍，确保不会因为定位误差而失败
        goal_tol = JointTolerance()
        goal_tol.name = joint_name
        # 至少 0.2 弧度，或最大变化的 1.5 倍
        goal_tol.position = max(0.2, max_position_change * 1.5)
        goal_tol.velocity = 0.5   # 0.5 rad/s 容差
        goal.goal_tolerance.append(goal_tol)

    # 创建轨迹点：起始点（当前位置）
    start_point = JointTrajectoryPoint()
    start_point.positions = current_positions
    start_point.velocities = [0.0] * len(joint_names)
    start_point.time_from_start = rospy.Duration(0.0)

    # 创建多个中间点，使轨迹更平滑（避免一顿一顿的感觉）
    max_change = max([abs(t - c)
                     for t, c in zip(target_positions, current_positions)])

    # 根据位置变化自动调整执行时间（大范围运动需要更长时间）
    # 速度限制：大约 0.5 rad/s，确保运动平滑
    duration = max(base_duration, max_change / 0.5)
    rospy.loginfo(f"位置变化: {max_change:.3f} 弧度，执行时间: {duration:.2f} 秒")

    # 根据位置变化大小决定轨迹点的数量
    if max_change > 0.5:  # 大范围运动，需要更多点
        num_points = 5  # 包括起始点和终点，共5个点
        rospy.loginfo(f"生成平滑轨迹（{num_points}个点）")
    elif max_change > 0.2:  # 中等范围运动
        num_points = 3  # 包括起始点和终点，共3个点
        rospy.loginfo(f"生成平滑轨迹（{num_points}个点）")
    else:  # 小范围运动
        num_points = 2  # 只有起始点和终点
        rospy.loginfo(f"生成简单轨迹（{num_points}个点）")

    # 生成轨迹点
    trajectory_points = []
    for i in range(num_points):
        point = JointTrajectoryPoint()
        if i == 0:
            # 起始点
            point.positions = current_positions
            point.time_from_start = rospy.Duration(0.0)
        elif i == num_points - 1:
            # 终点
            point.positions = target_positions
            point.time_from_start = rospy.Duration(duration)
        else:
            # 中间点：线性插值
            alpha = float(i) / float(num_points - 1)  # 0 到 1 之间的插值系数
            point.positions = [
                c + alpha * (t - c)
                for c, t in zip(current_positions, target_positions)
            ]
            point.time_from_start = rospy.Duration(duration * alpha)

        # 计算速度（使运动更平滑）
        if i == 0:
            # 起始速度为零
            point.velocities = [0.0] * len(joint_names)
        elif i == num_points - 1:
            # 终点速度为零
            point.velocities = [0.0] * len(joint_names)
        else:
            # 中间点的速度：根据相邻点计算，使运动更平滑
            if num_points > 2:
                # 计算当前点与前后点的位置差和时间差
                prev_alpha = float(i - 1) / float(num_points - 1)
                next_alpha = float(i + 1) / float(num_points - 1)
                dt = duration * (next_alpha - prev_alpha)

                if dt > 0:
                    # 计算速度：位置变化 / 时间
                    prev_pos = [
                        c + prev_alpha * (t - c)
                        for c, t in zip(current_positions, target_positions)
                    ]
                    next_pos = [
                        c + next_alpha * (t - c)
                        for c, t in zip(current_positions, target_positions)
                    ]
                    point.velocities = [
                        (next_pos[j] - prev_pos[j]) / dt
                        for j in range(len(joint_names))
                    ]
                else:
                    point.velocities = [0.0] * len(joint_names)
            else:
                point.velocities = [0.0] * len(joint_names)

        trajectory_points.append(point)

    goal.trajectory.points = trajectory_points

    goal.trajectory.header.stamp = rospy.Time.now()

    # 发送目标
    rospy.loginfo(f"发送轨迹目标到 {controller_name}...")
    rospy.loginfo(f"关节: {joint_names}")
    rospy.loginfo(f"起始位置: {[f'{p:.3f}' for p in current_positions]}")
    rospy.loginfo(f"目标位置: {[f'{p:.3f}' for p in target_positions]}")
    max_change = max([abs(t - c)
                     for t, c in zip(target_positions, current_positions)])
    rospy.loginfo(
        f"位置变化: {[f'{t-p:.3f}' for t, p in zip(target_positions, current_positions)]}")
    rospy.loginfo(f"最大位置变化: {max_change:.3f} 弧度 (约{max_change*57.3:.1f}度)")
    rospy.loginfo(f"执行时间: {duration} 秒")
    rospy.loginfo(f"路径容差: {max(0.3, max_change * 3.0):.3f} 弧度")
    goal_tol_value = max(0.2, max_change * 1.5)
    rospy.loginfo(
        f"目标容差: {goal_tol_value:.3f} 弧度 (约{goal_tol_value*57.3:.1f}度)")
    client.send_goal(goal)

    # 等待执行完成
    rospy.loginfo("等待轨迹执行完成...")
    client.wait_for_result()

    state = client.get_state()
    result = client.get_result()

    # 状态码含义
    status_names = {
        actionlib.GoalStatus.PENDING: "PENDING",
        actionlib.GoalStatus.ACTIVE: "ACTIVE",
        actionlib.GoalStatus.PREEMPTED: "PREEMPTED",
        actionlib.GoalStatus.SUCCEEDED: "SUCCEEDED",
        actionlib.GoalStatus.ABORTED: "ABORTED",
        actionlib.GoalStatus.REJECTED: "REJECTED",
        actionlib.GoalStatus.PREEMPTING: "PREEMPTING",
        actionlib.GoalStatus.RECALLING: "RECALLING",
        actionlib.GoalStatus.RECALLED: "RECALLED",
        actionlib.GoalStatus.LOST: "LOST"
    }

    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("✓ 轨迹执行成功！电机应该已经移动到目标位置")
        return True
    else:
        status_name = status_names.get(state, f"UNKNOWN({state})")
        rospy.logerr(f"✗ 轨迹执行失败")
        rospy.logerr(f"  状态码: {state} ({status_name})")

        if client.get_goal_status_text():
            rospy.logerr(f"  状态文本: {client.get_goal_status_text()}")

        # 获取详细的错误信息
        if result:
            if hasattr(result, 'error_code') and result.error_code != 0:
                error_codes = {
                    -1: "SUCCESSFUL",
                    0: "INVALID_GOAL",
                    1: "INVALID_JOINTS",
                    2: "OLD_HEADER_TIMESTAMP",
                    3: "PATH_TOLERANCE_VIOLATED",
                    4: "GOAL_TOLERANCE_VIOLATED"
                }
                error_code_name = error_codes.get(
                    result.error_code, f"UNKNOWN({result.error_code})")
                rospy.logerr(
                    f"  错误代码: {result.error_code} ({error_code_name})")

            if hasattr(result, 'error_string') and result.error_string:
                rospy.logerr(f"  错误字符串: {result.error_string}")

        # 提供诊断建议
        rospy.logerr("\n诊断建议：")
        rospy.logerr("1. 检查控制器是否正在运行：")
        rospy.logerr(
            f"   rosservice call /motor_hw_interface/hardware/controller_manager/list_controllers")
        rospy.logerr("2. 检查关节名称是否匹配：")
        rospy.logerr(f"   rosparam get /motor_hw_interface/hardware/joints")
        rospy.logerr(f"   期望的关节: {joint_names}")
        rospy.logerr("3. 检查关节状态是否正常：")
        rospy.logerr(
            "   rostopic echo /motor_hw_interface/hardware/joint_states")
        rospy.logerr("4. 检查 action 反馈：")
        rospy.logerr(f"   rostopic echo {action_topic}/feedback")
        rospy.logerr("5. 检查控制器日志（查看启动终端的输出）")

        return False


if __name__ == '__main__':
    try:
        success = send_trajectory()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
        sys.exit(1)
