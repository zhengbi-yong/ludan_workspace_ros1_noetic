#!/usr/bin/env python3
"""
检查硬件连接和电机反馈
"""

import rospy
import sys
import os


def check_serial_port(port="/dev/mcu"):
    """检查串口是否存在"""
    print(f"\n1. 检查串口 {port}...")
    if os.path.exists(port):
        print(f"   ✓ 串口文件存在")

        # 检查权限
        if os.access(port, os.R_OK | os.W_OK):
            print(f"   ✓ 串口有读写权限")
            return True
        else:
            print(f"   ✗ 串口无读写权限")
            print(f"   解决方案: sudo chmod a+rw {port}")
            return False
    else:
        print(f"   ✗ 串口文件不存在")
        print(f"   检查: ls -l /dev/ttyACM*")
        return False


def check_motor_states():
    """检查电机反馈"""
    print("\n2. 检查电机反馈话题...")
    try:
        from damiao_motor_control_board_serial.msg import MotorStates

        print("   等待电机反馈数据（5秒超时）...")
        msg = rospy.wait_for_message(
            '/motor_hw_interface/motor_states', MotorStates, timeout=5.0)

        print(f"   ✓ 收到电机反馈!")
        print(f"   电机数量: {len(msg.motors)}")

        # 显示前几个电机的状态
        print("   前5个电机状态:")
        for i, motor in enumerate(msg.motors[:5]):
            print(
                f"     电机 {motor.id}: pos={motor.pos:.3f}, vel={motor.vel:.3f}, tor={motor.tor:.3f}")

        # 检查是否有有效数据（非零位置）
        valid_count = sum(1 for m in msg.motors if abs(m.pos) > 0.001)
        print(f"   有效反馈数量: {valid_count}/{len(msg.motors)}")

        return True
    except ImportError:
        print("   ⚠ 无法导入消息类型")
        return False
    except rospy.ROSException:
        print("   ✗ 无电机反馈数据（5秒超时）")
        print("   可能原因:")
        print("     - 串口未连接或未打开")
        print("     - 电机未上电")
        print("     - 硬件接口节点启动失败")
        print("     - 串口权限问题")
        return False


def check_hardware_interface_node():
    """检查硬件接口节点"""
    print("\n3. 检查硬件接口节点...")
    try:
        import rosnode

        nodes = rosnode.get_node_names()
        hw_node = '/motor_hw_interface/motor_hw_interface'

        if hw_node in nodes:
            print(f"   ✓ 节点存在: {hw_node}")

            # 检查节点是否响应
            try:
                node_info = rosnode.get_node_info(hw_node, timeout=2.0)
                print("   ✓ 节点响应正常")
                return True
            except:
                print("   ✗ 节点无响应")
                return False
        else:
            print(f"   ✗ 节点不存在: {hw_node}")
            print("   检查启动日志，确认节点是否成功启动")
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_serial_connection_in_logs():
    """提示检查日志"""
    print("\n4. 检查启动日志...")
    print("   查看启动日志中是否有以下信息:")
    print("     - 'MotorDriver constructor entered !!!'")
    print("     - 'feedback_loop started'")
    print("     - 'Failed to open motor serial port' (如果有这个，说明串口打开失败)")
    print("     - 串口相关的错误信息")


def main():
    rospy.init_node('check_hardware_connection', anonymous=True)

    print("=" * 60)
    print("硬件连接诊断")
    print("=" * 60)

    # 获取串口参数
    try:
        port = rospy.get_param(
            '/motor_hw_interface/motor_hw_interface/port', '/dev/mcu')
    except:
        port = '/dev/mcu'

    # 检查串口
    serial_ok = check_serial_port(port)

    # 检查节点
    node_ok = check_hardware_interface_node()

    # 检查电机反馈
    motor_states_ok = check_motor_states()

    # 提示检查日志
    check_serial_connection_in_logs()

    # 总结
    print("\n" + "=" * 60)
    print("诊断结果:")
    print(f"  串口: {'✓' if serial_ok else '✗'}")
    print(f"  节点: {'✓' if node_ok else '✗'}")
    print(f"  电机反馈: {'✓' if motor_states_ok else '✗'}")

    if not motor_states_ok:
        print("\n修复建议:")
        if not serial_ok:
            print(f"  1. 修复串口权限: sudo chmod a+rw {port}")
        if not node_ok:
            print("  2. 检查硬件接口节点启动日志")
        print("  3. 检查:")
        print("     - 串口线是否连接")
        print("     - 电机控制板是否上电")
        print("     - 串口设备名称是否正确（可能是 /dev/ttyUSB0 或其他）")
        print("  4. 查看启动日志中的错误信息")

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
