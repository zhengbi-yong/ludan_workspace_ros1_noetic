#!/usr/bin/env python3
"""
诊断 joint_states 无数据的问题
"""

import rospy
import sys
import time

def check_joint_state_controller():
    """检查 joint_state_controller 状态"""
    print("\n1. 检查 joint_state_controller 状态...")
    try:
        from controller_manager_msgs.srv import ListControllers
        
        service_name = '/motor_hw_interface/controller_manager/list_controllers'
        
        print(f"   等待服务: {service_name}")
        rospy.wait_for_service(service_name, timeout=3.0)
        
        print("   调用服务（5秒超时）...")
        list_controllers = rospy.ServiceProxy(service_name, ListControllers)
        
        start_time = time.time()
        try:
            resp = list_controllers()
            
            if resp.controller:
                print(f"   ✓ 找到 {len(resp.controller)} 个控制器:")
                joint_state_ctrl = None
                for ctrl in resp.controller:
                    state = "运行中" if ctrl.state == "running" else ctrl.state
                    print(f"     - {ctrl.name}: {state}")
                    
                    if ctrl.name == "joint_state_controller":
                        joint_state_ctrl = ctrl
                
                if joint_state_ctrl:
                    if joint_state_ctrl.state == "running":
                        print("   ✓ joint_state_controller 正在运行")
                        return True
                    else:
                        print(f"   ✗ joint_state_controller 状态: {joint_state_ctrl.state}")
                        print("   可能原因: 控制器启动失败")
                        return False
                else:
                    print("   ✗ 未找到 joint_state_controller")
                    print("   可能原因: 控制器未加载")
                    return False
            else:
                print("   ✗ 没有控制器")
                return False
        except Exception as e:
            elapsed = time.time() - start_time
            if elapsed > 4.0:
                print(f"   ✗ 服务调用超时（{elapsed:.1f}秒）")
                return False
            else:
                raise
    except rospy.ROSException as e:
        print(f"   ✗ 服务不可用或超时: {str(e)}")
        return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_hardware_interface_joints():
    """检查硬件接口注册的关节"""
    print("\n2. 检查硬件接口注册的关节...")
    try:
        joints = rospy.get_param('/motor_hw_interface/joints', [])
        if joints:
            print(f"   ✓ 找到 {len(joints)} 个关节配置")
            
            # 显示前几个关节
            print("   前5个关节:")
            for i, joint in enumerate(joints[:5]):
                if isinstance(joint, dict):
                    name = joint.get('name', 'unknown')
                    motor_id = joint.get('id', -1)
                    print(f"     {name} -> 电机ID {motor_id}")
            
            return True
        else:
            print("   ✗ 关节配置未加载")
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_motor_states():
    """检查电机反馈"""
    print("\n3. 检查电机反馈...")
    try:
        from damiao_motor_control_board_serial.msg import MotorStates
        
        print("   等待电机反馈数据（3秒超时）...")
        msg = rospy.wait_for_message('/motor_hw_interface/motor_hw_interface/motor_states', MotorStates, timeout=3.0)
        
        print(f"   ✓ 收到电机反馈!")
        print(f"   电机数量: {len(msg.motors)}")
        
        # 检查是否有有效数据
        valid_count = 0
        for motor in msg.motors:
            if abs(motor.pos) > 0.001 or abs(motor.vel) > 0.001:
                valid_count += 1
        
        print(f"   有效反馈数量: {valid_count}/{len(msg.motors)}")
        
        return True
    except rospy.ROSException:
        print("   ✗ 无电机反馈数据（3秒超时）")
        return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_joint_states_topic():
    """检查 joint_states 话题"""
    print("\n4. 检查 joint_states 话题...")
    try:
        from sensor_msgs.msg import JointState
        
        # 检查原始话题
        print("   检查 /motor_hw_interface/joint_states...")
        try:
            msg = rospy.wait_for_message('/motor_hw_interface/joint_states', JointState, timeout=3.0)
            print(f"   ✓ 收到数据!")
            print(f"   关节数量: {len(msg.name)}")
            if len(msg.name) > 0:
                print("   前5个关节:")
                for i in range(min(5, len(msg.name))):
                    print(f"     {msg.name[i]}: pos={msg.position[i]:.3f}, vel={msg.velocity[i]:.3f}")
            return True
        except rospy.ROSException:
            print("   ✗ /motor_hw_interface/joint_states 无数据（3秒超时）")
            
            # 检查话题是否存在
            try:
                import rostopic
                topics = rostopic.get_topic_list()
                if '/motor_hw_interface/joint_states' in [t[0] for t in topics[0]]:
                    print("   ⚠ 话题存在但无数据")
                    print("   可能原因: joint_state_controller 未运行或无法读取关节数据")
                else:
                    print("   ✗ 话题不存在")
            except:
                pass
            
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_topic_publishers():
    """检查话题的发布者"""
    print("\n5. 检查话题发布者...")
    try:
        import rostopic
        
        # 检查 /motor_hw_interface/joint_states 的发布者
        print("   检查 /motor_hw_interface/joint_states 的发布者...")
        try:
            pub_info = rostopic.get_topic_info('/motor_hw_interface/joint_states', blocking=False)
            if pub_info and len(pub_info[0]) > 0:
                print(f"   ✓ 有发布者: {pub_info[0]}")
            else:
                print("   ✗ 无发布者")
                print("   可能原因: joint_state_controller 未运行")
        except Exception as e:
            print(f"   ✗ 无法获取发布者信息: {str(e)}")
    except Exception as e:
        print(f"   ⚠ 无法检查发布者: {str(e)}")


def main():
    rospy.init_node('diagnose_joint_states_issue', anonymous=True)
    
    print("=" * 60)
    print("诊断 joint_states 无数据问题")
    print("=" * 60)
    
    # 等待参数服务器就绪
    rospy.sleep(0.5)
    
    # 检查各项
    joints_ok = check_hardware_interface_joints()
    motor_states_ok = check_motor_states()
    controller_ok = check_joint_state_controller()
    joint_states_ok = check_joint_states_topic()
    check_topic_publishers()
    
    # 总结
    print("\n" + "=" * 60)
    print("诊断结果:")
    print(f"  关节配置: {'✓' if joints_ok else '✗'}")
    print(f"  电机反馈: {'✓' if motor_states_ok else '✗'}")
    print(f"  控制器状态: {'✓' if controller_ok else '✗'}")
    print(f"  joint_states: {'✓' if joint_states_ok else '✗'}")
    
    print("\n问题分析:")
    if not controller_ok:
        print("  ✗ joint_state_controller 未运行或启动失败")
        print("   解决方案:")
        print("     1. 检查启动日志，查看控制器启动错误")
        print("     2. 尝试手动启动控制器:")
        print("        rosservice call /motor_hw_interface/controller_manager/load_controller 'joint_state_controller'")
        print("        rosservice call /motor_hw_interface/controller_manager/switch_controller '{start_controllers: ['joint_state_controller'], stop_controllers: [], strictness: 2}'")
    elif not joint_states_ok:
        print("  ✗ joint_state_controller 运行中但无数据")
        print("   可能原因:")
        print("     1. 硬件接口没有正确注册关节到 JointStateInterface")
        print("     2. 控制器无法从硬件接口读取关节数据")
        print("     3. 关节名称不匹配")
        print("   解决方案:")
        print("     1. 检查硬件接口是否正确注册了关节")
        print("     2. 检查控制器配置是否正确")
        print("     3. 查看硬件接口节点的日志")
    
    print("=" * 60)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断")
    except Exception as e:
        print(f"\n错误: {str(e)}")
        import traceback
        traceback.print_exc()

