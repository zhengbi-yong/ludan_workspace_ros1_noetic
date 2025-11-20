#!/usr/bin/env python3
"""
MoveIt 规划和执行测试脚本
验证规划是否成功以及电机是否能随之运动
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import time


def test_planning_and_execution(group_name="left_arm"):
    """
    测试规划和执行

    Args:
        group_name: 规划组名称
    """
    # 初始化 MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_moveit_execution', anonymous=True)

    rospy.loginfo("=" * 60)
    rospy.loginfo("MoveIt 规划和执行测试")
    rospy.loginfo("=" * 60)

    # 等待 MoveIt 服务
    rospy.loginfo("等待 MoveIt 服务...")
    rospy.sleep(2.0)

    try:
        # 创建规划组
        rospy.loginfo(f"\n初始化规划组: {group_name}")
        group = moveit_commander.MoveGroupCommander(group_name)

        # 设置规划参数
        group.set_planning_time(10.0)
        group.set_num_planning_attempts(5)
        group.set_max_velocity_scaling_factor(0.1)  # 降低速度以确保安全
        group.set_max_acceleration_scaling_factor(0.1)

        # 获取当前状态
        rospy.loginfo("\n1. 获取当前关节状态...")
        current_joint_values = group.get_current_joint_values()
        rospy.loginfo(
            f"   当前关节值: {[f'{v:.3f}' for v in current_joint_values]}")

        current_pose = group.get_current_pose()
        rospy.loginfo(f"   当前位置: x={current_pose.pose.position.x:.3f}, "
                      f"y={current_pose.pose.position.y:.3f}, "
                      f"z={current_pose.pose.position.z:.3f}")

        # 测试1: 关节空间规划
        rospy.loginfo("\n2. 测试关节空间规划...")
        target_joint_values = list(current_joint_values)

        # 稍微移动第一个关节（如果存在）
        if len(target_joint_values) > 0:
            target_joint_values[0] += 0.1  # 移动 0.1 弧度（约 5.7 度）
            rospy.loginfo(
                f"   目标关节值: {[f'{v:.3f}' for v in target_joint_values]}")

            group.set_joint_value_target(target_joint_values)

            rospy.loginfo("   开始规划...")
            plan = group.plan()

            if plan and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo(
                    f"   ✓ 规划成功！轨迹包含 {len(plan.joint_trajectory.points)} 个点")
                rospy.loginfo(
                    f"   轨迹时长: {plan.joint_trajectory.points[-1].time_from_start.to_sec():.2f} 秒")

                # 询问是否执行
                rospy.loginfo("\n   准备执行轨迹...")
                rospy.loginfo("   注意: 这将实际移动电机，请确保安全！")
                rospy.sleep(1.0)

                rospy.loginfo("   执行轨迹...")
                success = group.execute(plan, wait=True)

                if success:
                    rospy.loginfo("   ✓ 轨迹执行成功！")
                    rospy.sleep(1.0)

                    # 验证位置
                    new_joint_values = group.get_current_joint_values()
                    rospy.loginfo(
                        f"   执行后关节值: {[f'{v:.3f}' for v in new_joint_values]}")

                    # 检查是否真的移动了
                    diff = abs(new_joint_values[0] - current_joint_values[0])
                    if diff > 0.01:
                        rospy.loginfo(f"   ✓ 电机已移动！第一个关节移动了 {diff:.3f} 弧度")
                    else:
                        rospy.logwarn(f"   ⚠ 电机可能未移动，差值: {diff:.3f} 弧度")
                else:
                    rospy.logwarn("   ✗ 轨迹执行失败")
            else:
                rospy.logwarn("   ✗ 规划失败，无法生成有效轨迹")
        else:
            rospy.logwarn("   无法获取关节状态")

        # 测试2: 返回初始位置
        rospy.loginfo("\n3. 返回初始位置...")
        group.set_joint_value_target(current_joint_values)
        plan = group.plan()

        if plan and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("   规划返回路径...")
            success = group.execute(plan, wait=True)
            if success:
                rospy.loginfo("   ✓ 已返回初始位置")
            else:
                rospy.logwarn("   ✗ 返回失败")

        # 测试3: 检查控制器状态
        rospy.loginfo("\n4. 检查控制器状态...")
        try:
            import rosservice
            from controller_manager_msgs.srv import ListControllers

            list_controllers = rospy.ServiceProxy(
                '/motor_hw_interface/controller_manager/list_controllers',
                ListControllers
            )

            try:
                response = list_controllers()
                rospy.loginfo("   已加载的控制器:")
                for controller in response.controller:
                    state = "运行中" if controller.state == "running" else controller.state
                    rospy.loginfo(f"     - {controller.name}: {state}")
            except Exception as e:
                rospy.logwarn(f"   无法获取控制器状态: {str(e)}")
        except Exception as e:
            rospy.logwarn(f"   无法检查控制器状态: {str(e)}")

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("测试完成！")
        rospy.loginfo("=" * 60)

    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt 已关闭")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="测试 MoveIt 规划和执行",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        "--group",
        type=str,
        default="left_arm",
        help="规划组名称 (默认: left_arm)",
        choices=["left_arm", "right_arm", "left_leg",
                 "right_leg", "neck", "upper_body"]
    )

    args = parser.parse_args()

    test_planning_and_execution(args.group)


if __name__ == '__main__':
    main()
