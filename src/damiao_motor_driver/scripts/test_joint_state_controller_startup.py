#!/usr/bin/env python3
"""
测试 joint_state_controller 启动
"""

import rospy
import sys
import time

def test_controller_startup():
    """测试控制器启动"""
    rospy.init_node('test_joint_state_controller_startup', anonymous=True)
    
    print("=" * 60)
    print("测试 joint_state_controller 启动")
    print("=" * 60)
    
    # 等待参数服务器就绪
    rospy.sleep(0.5)
    
    try:
        from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
        
        load_service = '/motor_hw_interface/controller_manager/load_controller'
        switch_service = '/motor_hw_interface/controller_manager/switch_controller'
        list_service = '/motor_hw_interface/controller_manager/list_controllers'
        
        print(f"\n1. 等待服务: {load_service}")
        rospy.wait_for_service(load_service, timeout=3.0)
        print("   ✓ 服务可用")
        
        print(f"\n2. 检查当前控制器状态...")
        try:
            list_controllers = rospy.ServiceProxy(list_service, ListControllers)
            resp = list_controllers()
            print(f"   当前控制器数量: {len(resp.controller)}")
            for ctrl in resp.controller:
                print(f"     - {ctrl.name}: {ctrl.state}")
        except Exception as e:
            print(f"   ⚠ 无法获取控制器列表: {str(e)}")
        
        print(f"\n3. 尝试加载控制器...")
        load_controller = rospy.ServiceProxy(load_service, LoadController)
        try:
            resp = load_controller("joint_state_controller")
            if resp.ok:
                print("   ✓ 控制器加载成功")
            else:
                print(f"   ✗ 控制器加载失败: {resp.error_message}")
                return False
        except Exception as e:
            print(f"   ✗ 加载失败: {str(e)}")
            return False
        
        rospy.sleep(0.5)
        
        print(f"\n4. 尝试启动控制器...")
        switch_controller = rospy.ServiceProxy(switch_service, SwitchController)
        try:
            resp = switch_controller(
                ["joint_state_controller"],  # start_controllers
                [],  # stop_controllers
                2,  # strictness: 2 = STRICT
                False,  # start_asap
                0.0  # timeout
            )
            if resp.ok:
                print("   ✓ 控制器启动成功")
            else:
                print(f"   ✗ 控制器启动失败: {resp.error_message}")
                return False
        except Exception as e:
            print(f"   ✗ 启动失败: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
        
        rospy.sleep(1.0)
        
        print(f"\n5. 检查控制器状态...")
        try:
            resp = list_controllers()
            for ctrl in resp.controller:
                if ctrl.name == "joint_state_controller":
                    print(f"   {ctrl.name}: {ctrl.state}")
                    if ctrl.state == "running":
                        print("   ✓ 控制器正在运行")
                        return True
                    else:
                        print(f"   ✗ 控制器状态: {ctrl.state}")
                        return False
        except Exception as e:
            print(f"   ⚠ 无法获取控制器状态: {str(e)}")
        
        return False
        
    except rospy.ROSException as e:
        print(f"   ✗ 服务不可用: {str(e)}")
        return False
    except Exception as e:
        print(f"   ✗ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def check_joint_states():
    """检查 joint_states 话题"""
    print("\n6. 检查 joint_states 话题...")
    try:
        from sensor_msgs.msg import JointState
        
        print("   等待数据（3秒超时）...")
        msg = rospy.wait_for_message('/motor_hw_interface/joint_states', JointState, timeout=3.0)
        
        print(f"   ✓ 收到数据!")
        print(f"   关节数量: {len(msg.name)}")
        if len(msg.name) > 0:
            print("   前5个关节:")
            for i in range(min(5, len(msg.name))):
                print(f"     {msg.name[i]}: pos={msg.position[i]:.3f}, vel={msg.velocity[i]:.3f}")
        
        return True
    except rospy.ROSException:
        print("   ✗ 无数据（3秒超时）")
        return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def main():
    print("=" * 60)
    print("测试 joint_state_controller 启动")
    print("=" * 60)
    
    controller_ok = test_controller_startup()
    
    if controller_ok:
        joint_states_ok = check_joint_states()
    else:
        joint_states_ok = False
    
    print("\n" + "=" * 60)
    print("测试结果:")
    print(f"  控制器: {'✓' if controller_ok else '✗'}")
    print(f"  joint_states: {'✓' if joint_states_ok else '✗'}")
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

