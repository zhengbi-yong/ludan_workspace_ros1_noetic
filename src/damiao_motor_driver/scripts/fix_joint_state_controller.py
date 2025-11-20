#!/usr/bin/env python3
"""
修复 joint_state_controller 问题
"""

import rospy
import sys
import time

def check_controller_status():
    """检查控制器状态（使用超时避免卡住）"""
    print("\n1. 检查控制器状态...")
    try:
        from controller_manager_msgs.srv import ListControllers
        
        service_name = '/motor_hw_interface/controller_manager/list_controllers'
        
        print(f"   等待服务: {service_name}")
        rospy.wait_for_service(service_name, timeout=3.0)
        
        print("   调用服务（5秒超时）...")
        list_controllers = rospy.ServiceProxy(service_name, ListControllers)
        
        # 使用超时机制
        start_time = time.time()
        try:
            resp = list_controllers()
            
            if resp.controller:
                print(f"   ✓ 找到 {len(resp.controller)} 个控制器:")
                for ctrl in resp.controller:
                    state = "运行中" if ctrl.state == "running" else ctrl.state
                    print(f"     - {ctrl.name}: {state}")
                    
                    if ctrl.name == "joint_state_controller":
                        if ctrl.state == "running":
                            print("   ✓ joint_state_controller 正在运行")
                            return True
                        else:
                            print(f"   ✗ joint_state_controller 状态: {ctrl.state}")
                            return False
                
                print("   ✗ 未找到 joint_state_controller")
                return False
            else:
                print("   ✗ 没有控制器")
                return False
        except rospy.ROSException as e:
            elapsed = time.time() - start_time
            if elapsed > 4.0:
                print(f"   ✗ 服务调用超时（{elapsed:.1f}秒）")
                print("   可能原因: controller_manager 无响应")
                return False
            else:
                raise
    except rospy.ROSException as e:
        print(f"   ✗ 服务不可用或超时: {str(e)}")
        return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def load_and_start_controller():
    """加载并启动 joint_state_controller"""
    print("\n2. 尝试加载并启动 joint_state_controller...")
    try:
        from controller_manager_msgs.srv import LoadController, SwitchController
        
        load_service = '/motor_hw_interface/controller_manager/load_controller'
        switch_service = '/motor_hw_interface/controller_manager/switch_controller'
        
        print(f"   等待服务: {load_service}")
        rospy.wait_for_service(load_service, timeout=3.0)
        rospy.wait_for_service(switch_service, timeout=3.0)
        
        load_controller = rospy.ServiceProxy(load_service, LoadController)
        switch_controller = rospy.ServiceProxy(switch_service, SwitchController)
        
        # 尝试加载
        print("   加载控制器...")
        try:
            resp = load_controller("joint_state_controller")
            if resp.ok:
                print("   ✓ 控制器加载成功")
            else:
                print(f"   ✗ 控制器加载失败: {resp.ok}")
                return False
        except Exception as e:
            print(f"   ✗ 加载失败: {str(e)}")
            # 可能已经加载了，继续尝试启动
        
        # 尝试启动
        print("   启动控制器...")
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
                return True
            else:
                print(f"   ✗ 控制器启动失败: {resp.ok}")
                return False
        except Exception as e:
            print(f"   ✗ 启动失败: {str(e)}")
            return False
    except rospy.ROSException as e:
        print(f"   ✗ 服务不可用: {str(e)}")
        return False
    except Exception as e:
        print(f"   ✗ 操作失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def check_joint_states_topic():
    """检查 joint_states 话题"""
    print("\n3. 检查 joint_states 话题...")
    try:
        from sensor_msgs.msg import JointState
        
        print("   等待数据（5秒超时）...")
        msg = rospy.wait_for_message('/motor_hw_interface/joint_states', JointState, timeout=5.0)
        
        print(f"   ✓ 收到 joint_states 数据!")
        print(f"   关节数量: {len(msg.name)}")
        
        if len(msg.name) > 0:
            print("   前5个关节:")
            for i in range(min(5, len(msg.name))):
                print(f"     {msg.name[i]}: pos={msg.position[i]:.3f}, vel={msg.velocity[i]:.3f}")
        
        return True
    except rospy.ROSException:
        print("   ✗ 无 joint_states 数据（5秒超时）")
        return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def main():
    rospy.init_node('fix_joint_state_controller', anonymous=True)
    
    print("=" * 60)
    print("修复 joint_state_controller")
    print("=" * 60)
    
    # 等待参数服务器就绪
    rospy.sleep(0.5)
    
    # 检查控制器状态
    controller_running = check_controller_status()
    
    # 如果控制器未运行，尝试启动
    if not controller_running:
        print("\n控制器未运行，尝试启动...")
        if load_and_start_controller():
            rospy.sleep(1.0)  # 等待控制器启动
            controller_running = check_controller_status()
    
    # 检查 joint_states 话题
    if controller_running:
        joint_states_ok = check_joint_states_topic()
    else:
        print("\n⚠ 控制器未运行，跳过 joint_states 检查")
        joint_states_ok = False
    
    # 总结
    print("\n" + "=" * 60)
    print("修复结果:")
    print(f"  控制器状态: {'✓' if controller_running else '✗'}")
    print(f"  joint_states: {'✓' if joint_states_ok else '✗'}")
    
    if not controller_running:
        print("\n修复建议:")
        print("  1. 检查启动日志，查看 controller_spawner 是否有错误")
        print("  2. 检查控制器配置文件是否正确")
        print("  3. 检查硬件接口是否正确注册了关节")
        print("  4. 尝试手动启动:")
        print("     rosservice call /motor_hw_interface/controller_manager/load_controller 'joint_state_controller'")
        print("     rosservice call /motor_hw_interface/controller_manager/switch_controller '{start_controllers: ['joint_state_controller'], stop_controllers: [], strictness: 2}'")
    
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

