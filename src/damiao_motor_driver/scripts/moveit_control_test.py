#!/usr/bin/env python3
"""
简单的 MoveIt 控制测试脚本
用于通过程序接口控制电机运动
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    # 初始化 MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_control_test', anonymous=True)
    
    # 等待 MoveIt 服务可用
    rospy.loginfo("等待 MoveIt 服务...")
    rospy.sleep(2.0)
    
    try:
        # 创建规划组（例如左臂）
        group_name = "left_arm"
        rospy.loginfo(f"初始化规划组: {group_name}")
        group = moveit_commander.MoveGroupCommander(group_name)
        
        # 设置规划参数
        group.set_planning_time(10.0)
        group.set_num_planning_attempts(5)
        group.set_max_velocity_scaling_factor(0.1)  # 降低速度以确保安全
        group.set_max_acceleration_scaling_factor(0.1)
        
        # 获取当前状态
        rospy.loginfo("获取当前关节状态...")
        current_joint_values = group.get_current_joint_values()
        rospy.loginfo(f"当前关节值: {current_joint_values}")
        
        # 设置目标关节位置（稍微移动）
        target_joint_values = list(current_joint_values)
        if len(target_joint_values) > 0:
            target_joint_values[0] += 0.1  # 第一个关节移动 0.1 弧度
            rospy.loginfo(f"目标关节值: {target_joint_values}")
            
            # 设置目标
            group.set_joint_value_target(target_joint_values)
            
            # 规划轨迹
            rospy.loginfo("开始规划轨迹...")
            plan = group.plan()
            
            if plan and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo(f"规划成功！轨迹包含 {len(plan.joint_trajectory.points)} 个点")
                
                # 执行轨迹
                rospy.loginfo("执行轨迹...")
                success = group.execute(plan, wait=True)
                
                if success:
                    rospy.loginfo("轨迹执行成功！")
                else:
                    rospy.logwarn("轨迹执行失败")
            else:
                rospy.logwarn("规划失败，无法生成有效轨迹")
        else:
            rospy.logwarn("无法获取当前关节状态")
            
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt 控制测试完成")

if __name__ == '__main__':
    main()

