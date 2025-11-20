#!/usr/bin/env python3
"""
检查和修复 joint_states 问题
"""

import rospy
import sys
from controller_manager_msgs.srv import ListControllers, SwitchController, LoadController
from std_srvs.srv import Trigger

def check_controllers():
    """检查控制器状态"""
    rospy.loginfo("检查控制器状态...")
    
    try:
        list_controllers = rospy.ServiceProxy(
            '/motor_hw_interface/controller_manager/list_controllers',
            ListControllers
        )
        
        # 设置超时
        rospy.wait_for_service('/motor_hw_interface/controller_manager/list_controllers', timeout=3.0)
        
        try:
            response = list_controllers()
            rospy.loginfo(f"找到 {len(response.controller)} 个控制器:")
            for controller in response.controller:
                rospy.loginfo(f"  - {controller.name}: {controller.state}")
                if controller.name == "joint_state_controller":
                    if controller.state == "running":
                        rospy.loginfo("    ✓ joint_state_controller 正在运行")
                        return True
                    else:
                        rospy.logwarn(f"    ✗ joint_state_controller 状态: {controller.state}")
                        return False
            rospy.logwarn("  ✗ 未找到 joint_state_controller")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"调用服务失败: {str(e)}")
            return False
    except rospy.ROSException as e:
        rospy.logerr(f"服务不可用: {str(e)}")
        return False


def load_and_start_controller():
    """加载并启动 joint_state_controller"""
    rospy.loginfo("尝试加载并启动 joint_state_controller...")
    
    try:
        # 加载控制器
        load_controller = rospy.ServiceProxy(
            '/motor_hw_interface/controller_manager/load_controller',
            LoadController
        )
        
        rospy.wait_for_service('/motor_hw_interface/controller_manager/load_controller', timeout=3.0)
        
        try:
            response = load_controller('joint_state_controller')
            if response.ok:
                rospy.loginfo("  ✓ 控制器加载成功")
            else:
                rospy.logwarn(f"  ✗ 控制器加载失败: {response.error_message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"加载控制器失败: {str(e)}")
            return False
        
        rospy.sleep(0.5)
        
        # 启动控制器
        switch_controller = rospy.ServiceProxy(
            '/motor_hw_interface/controller_manager/switch_controller',
            SwitchController
        )
        
        rospy.wait_for_service('/motor_hw_interface/controller_manager/switch_controller', timeout=3.0)
        
        try:
            response = switch_controller(
                start_controllers=['joint_state_controller'],
                stop_controllers=[],
                strictness=2
            )
            if response.ok:
                rospy.loginfo("  ✓ 控制器启动成功")
                return True
            else:
                rospy.logwarn(f"  ✗ 控制器启动失败: {response.error_message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"启动控制器失败: {str(e)}")
            return False
            
    except rospy.ROSException as e:
        rospy.logerr(f"服务不可用: {str(e)}")
        return False


def check_joint_states_topic():
    """检查 joint_states 话题"""
    rospy.loginfo("检查 joint_states 话题...")
    
    import rostopic
    
    # 检查原始话题
    try:
        topic_type, topic_name, _ = rostopic.get_topic_type('/motor_hw_interface/joint_states')
        if topic_type:
            rospy.loginfo(f"  ✓ /motor_hw_interface/joint_states 存在 (类型: {topic_type})")
            
            # 尝试接收一条消息
            from sensor_msgs.msg import JointState
            try:
                msg = rospy.wait_for_message('/motor_hw_interface/joint_states', JointState, timeout=2.0)
                rospy.loginfo(f"  ✓ 收到数据: {len(msg.name)} 个关节")
                rospy.loginfo(f"    关节名称: {msg.name[:5]}..." if len(msg.name) > 5 else f"    关节名称: {msg.name}")
                return True
            except rospy.ROSException:
                rospy.logwarn("  ✗ 话题存在但无数据")
                return False
        else:
            rospy.logwarn("  ✗ /motor_hw_interface/joint_states 不存在")
            return False
    except Exception as e:
        rospy.logerr(f"  ✗ 检查话题失败: {str(e)}")
        return False


def main():
    rospy.init_node('check_and_fix_joint_states', anonymous=True)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Joint States 检查和修复工具")
    rospy.loginfo("=" * 60)
    
    # 1. 检查话题
    topic_ok = check_joint_states_topic()
    
    # 2. 检查控制器
    controller_ok = check_controllers()
    
    # 3. 如果控制器未运行，尝试启动
    if not controller_ok:
        rospy.loginfo("\n尝试修复...")
        if load_and_start_controller():
            rospy.sleep(1.0)
            controller_ok = check_controllers()
            if controller_ok:
                rospy.sleep(1.0)
                topic_ok = check_joint_states_topic()
    
    # 总结
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("检查结果:")
    rospy.loginfo(f"  话题状态: {'✓ 正常' if topic_ok else '✗ 异常'}")
    rospy.loginfo(f"  控制器状态: {'✓ 正常' if controller_ok else '✗ 异常'}")
    
    if topic_ok and controller_ok:
        rospy.loginfo("\n✓ 系统正常，可以开始测试 MoveIt")
    else:
        rospy.logwarn("\n✗ 系统异常，请检查:")
        if not controller_ok:
            rospy.logwarn("  - joint_state_controller 未运行")
            rospy.logwarn("  - 检查启动日志中是否有错误")
        if not topic_ok:
            rospy.logwarn("  - joint_states 话题无数据")
            rospy.logwarn("  - 检查硬件接口是否正常")
            rospy.logwarn("  - 检查关节映射配置是否正确")
    
    rospy.loginfo("=" * 60)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("中断")
    except Exception as e:
        rospy.logerr(f"错误: {str(e)}")
        import traceback
        traceback.print_exc()

