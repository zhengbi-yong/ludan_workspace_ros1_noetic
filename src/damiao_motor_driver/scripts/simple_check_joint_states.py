#!/usr/bin/env python3
"""
简单的 joint_states 检查脚本（避免服务调用卡住）
"""

import rospy
import sys
from sensor_msgs.msg import JointState

def check_joint_states():
    """检查 joint_states 话题"""
    rospy.init_node('simple_check_joint_states', anonymous=True)
    
    print("=" * 60)
    print("Joint States 简单检查")
    print("=" * 60)
    
    # 检查原始话题
    print("\n1. 检查 /motor_hw_interface/joint_states...")
    try:
        msg = rospy.wait_for_message('/motor_hw_interface/joint_states', JointState, timeout=3.0)
        print(f"   ✓ 收到数据!")
        print(f"   关节数量: {len(msg.name)}")
        print(f"   关节名称: {msg.name[:10] if len(msg.name) > 10 else msg.name}")
        print(f"   位置值: {[f'{p:.3f}' for p in msg.position[:5]]}...")
        return True
    except rospy.ROSException:
        print("   ✗ 无数据（3秒超时）")
        print("   可能原因:")
        print("     - joint_state_controller 未运行")
        print("     - 硬件接口没有关节数据")
        return False
    
    # 检查重映射话题
    print("\n2. 检查 /joint_states...")
    try:
        msg = rospy.wait_for_message('/joint_states', JointState, timeout=2.0)
        print(f"   ✓ 收到数据!")
        return True
    except rospy.ROSException:
        print("   ✗ 无数据")
        return False


def check_motor_states():
    """检查电机反馈"""
    print("\n3. 检查电机反馈 /motor_hw_interface/motor_states...")
    try:
        from damiao_motor_control_board_serial.msg import MotorStates
        msg = rospy.wait_for_message('/motor_hw_interface/motor_states', MotorStates, timeout=3.0)
        print(f"   ✓ 收到电机反馈!")
        print(f"   电机数量: {len(msg.motors)}")
        if len(msg.motors) > 0:
            m = msg.motors[0]
            print(f"   示例电机 (ID={m.id}): pos={m.pos:.3f}, vel={m.vel:.3f}, tor={m.tor:.3f}")
        return True
    except ImportError:
        print("   ⚠ 无法导入消息类型，跳过")
        return False
    except rospy.ROSException:
        print("   ✗ 无电机反馈数据")
        return False


def check_joint_mapping():
    """检查关节映射配置"""
    print("\n4. 检查关节映射配置...")
    try:
        import rospy
        # 检查参数服务器
        if rospy.has_param('/motor_hw_interface/joints'):
            joints = rospy.get_param('/motor_hw_interface/joints')
            if joints and len(joints) > 0:
                print(f"   ✓ 找到关节配置: {len(joints)} 个关节")
                if isinstance(joints[0], dict):
                    print(f"   示例关节: name={joints[0].get('name', 'N/A')}, id={joints[0].get('id', 'N/A')}")
                else:
                    print(f"   示例关节: {joints[0]}")
                return True
            else:
                print("   ✗ 关节配置为空")
                return False
        else:
            print("   ✗ 未找到关节映射配置")
            print("   提示: 需要在启动时加载关节映射文件")
            print("   启动命令应包含: joint_mapping:=/path/to/joint_mapping.yaml")
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def main():
    try:
        # 检查 joint_states
        joint_states_ok = check_joint_states()
        
        # 检查电机反馈
        motor_states_ok = check_motor_states()
        
        # 检查关节映射
        mapping_ok = check_joint_mapping()
        
        # 总结
        print("\n" + "=" * 60)
        print("检查结果:")
        print(f"  joint_states: {'✓' if joint_states_ok else '✗'}")
        print(f"  motor_states: {'✓' if motor_states_ok else '✗'}")
        print(f"  关节映射: {'✓' if mapping_ok else '✗'}")
        
        if not joint_states_ok:
            print("\n建议:")
            if not mapping_ok:
                print("  1. 检查关节映射文件是否正确加载")
                print("     启动时添加: joint_mapping:=/path/to/joint_mapping.yaml")
            if not motor_states_ok:
                print("  2. 检查硬件连接和电机反馈")
            print("  3. 检查 joint_state_controller 是否运行:")
            print("     rosnode list | grep controller")
        
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n中断")
    except Exception as e:
        print(f"\n错误: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

