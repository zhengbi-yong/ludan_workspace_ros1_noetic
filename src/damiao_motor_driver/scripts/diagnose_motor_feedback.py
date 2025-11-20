#!/usr/bin/env python3
"""
诊断电机反馈问题
"""

import rospy
import sys
import os
import subprocess

def check_node_logs():
    """检查节点日志中的关键信息"""
    print("\n1. 检查节点日志...")
    print("   请查看启动日志中是否有以下信息:")
    print("   ✓ 'MotorDriver constructor entered !!!' - 驱动构造成功")
    print("   ✓ 'feedback_loop started' - 反馈循环启动")
    print("   ✗ 'Failed to open motor serial port after retries!' - 串口打开失败")
    print("   ✗ 'Serial IO exception' - 串口IO异常")
    print("   ✗ 'Serial exception' - 串口异常")
    print("   ✗ 'Motor feedback timeout' - 电机反馈超时")


def check_serial_port():
    """检查串口"""
    print("\n2. 检查串口...")
    try:
        port = rospy.get_param('/motor_hw_interface/motor_hw_interface/port', '/dev/ttyACM0')
    except:
        port = '/dev/ttyACM0'
    
    print(f"   配置的串口: {port}")
    
    if os.path.exists(port):
        print(f"   ✓ 串口文件存在")
        
        # 检查权限
        if os.access(port, os.R_OK | os.W_OK):
            print(f"   ✓ 串口有读写权限")
        else:
            print(f"   ✗ 串口无读写权限")
            print(f"   修复: sudo chmod a+rw {port}")
            return False
        
        # 检查是否有其他进程占用
        try:
            result = subprocess.run(['lsof', port], capture_output=True, text=True, timeout=2)
            if result.returncode == 0 and result.stdout:
                print(f"   ⚠ 串口被占用:")
                print(f"     {result.stdout.strip()}")
            else:
                print(f"   ✓ 串口未被占用")
        except:
            pass
        
        return True
    else:
        print(f"   ✗ 串口文件不存在")
        print(f"   检查可用串口: ls -l /dev/ttyACM* /dev/ttyUSB*")
        return False


def check_motor_states_topic():
    """检查电机状态话题"""
    print("\n3. 检查电机状态话题...")
    
    # 检查话题是否存在
    try:
        import rostopic
        topics = rostopic.get_topic_list()
        motor_states_topics = [t for t in topics[0] if 'motor_states' in t]
        
        if motor_states_topics:
            print(f"   ✓ 找到电机状态话题:")
            for topic in motor_states_topics:
                print(f"     - {topic}")
            
            # 检查是否有发布者
            for topic in motor_states_topics:
                try:
                    pub_info = rostopic.get_topic_info(topic, blocking=False)
                    if pub_info and len(pub_info[0]) > 0:
                        print(f"     ✓ {topic} 有发布者: {pub_info[0]}")
                    else:
                        print(f"     ✗ {topic} 无发布者")
                except:
                    pass
        else:
            print(f"   ✗ 未找到电机状态话题")
            print(f"   可能原因: 硬件接口节点未启动或启动失败")
            return False
    except Exception as e:
        print(f"   ⚠ 检查话题时出错: {str(e)}")
    
    # 尝试接收数据
    print("\n   尝试接收电机反馈数据（5秒超时）...")
    try:
        from damiao_motor_control_board_serial.msg import MotorStates
        msg = rospy.wait_for_message('/motor_hw_interface/motor_states', MotorStates, timeout=5.0)
        
        print(f"   ✓ 收到电机反馈!")
        print(f"   电机数量: {len(msg.motors)}")
        
        # 统计有效反馈
        valid_count = 0
        for motor in msg.motors:
            if abs(motor.pos) > 0.001 or abs(motor.vel) > 0.001:
                valid_count += 1
        
        print(f"   有效反馈数量: {valid_count}/{len(msg.motors)}")
        
        if valid_count == 0:
            print("   ⚠ 警告: 所有电机位置和速度都接近0")
            print("   可能原因:")
            print("     - 电机未上电")
            print("     - 电机未连接")
            print("     - 反馈数据解析错误")
        
        return True
    except ImportError:
        print("   ✗ 无法导入消息类型")
        return False
    except rospy.ROSException:
        print("   ✗ 无电机反馈数据（5秒超时）")
        return False


def check_hardware_interface_node():
    """检查硬件接口节点"""
    print("\n4. 检查硬件接口节点...")
    try:
        import rosnode
        
        nodes = rosnode.get_node_names()
        hw_node = '/motor_hw_interface/motor_hw_interface'
        
        if hw_node in nodes:
            print(f"   ✓ 节点存在: {hw_node}")
            
            # 检查节点信息
            try:
                node_info = rosnode.get_node_info(hw_node, timeout=2.0)
                print("   ✓ 节点响应正常")
                
                # 检查发布的话题
                if 'Publishers' in str(node_info):
                    print("   节点发布的话题:")
                    # 这里可以解析node_info获取发布的话题列表
                
                return True
            except Exception as e:
                print(f"   ✗ 节点无响应: {str(e)}")
                return False
        else:
            print(f"   ✗ 节点不存在: {hw_node}")
            print("   检查启动日志，确认节点是否成功启动")
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def check_joint_mapping():
    """检查关节映射"""
    print("\n5. 检查关节映射...")
    try:
        joints = rospy.get_param('/motor_hw_interface/joints', [])
        if joints:
            print(f"   ✓ 关节映射已加载: {len(joints)} 个关节")
            
            # 显示前几个关节
            print("   前5个关节:")
            for i, joint in enumerate(joints[:5]):
                if isinstance(joint, dict):
                    name = joint.get('name', 'unknown')
                    motor_id = joint.get('id', -1)
                    print(f"     {name} -> 电机ID {motor_id}")
            
            return True
        else:
            print("   ✗ 关节映射未加载")
            return False
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        return False


def main():
    rospy.init_node('diagnose_motor_feedback', anonymous=True)
    
    print("=" * 60)
    print("电机反馈问题诊断")
    print("=" * 60)
    
    # 等待参数服务器就绪
    rospy.sleep(0.5)
    
    # 检查各项
    serial_ok = check_serial_port()
    node_ok = check_hardware_interface_node()
    mapping_ok = check_joint_mapping()
    motor_states_ok = check_motor_states_topic()
    check_node_logs()
    
    # 总结
    print("\n" + "=" * 60)
    print("诊断结果:")
    print(f"  串口: {'✓' if serial_ok else '✗'}")
    print(f"  节点: {'✓' if node_ok else '✗'}")
    print(f"  关节映射: {'✓' if mapping_ok else '✗'}")
    print(f"  电机反馈: {'✓' if motor_states_ok else '✗'}")
    
    print("\n修复建议:")
    if not serial_ok:
        print("  1. 修复串口权限或检查串口设备")
    if not node_ok:
        print("  2. 检查硬件接口节点启动日志，查找错误信息")
    if not motor_states_ok:
        print("  3. 检查:")
        print("     - 串口线是否连接")
        print("     - 电机控制板是否上电")
        print("     - 串口设备名称是否正确")
        print("     - 查看启动日志中的串口打开错误")
    if not mapping_ok:
        print("  4. 确保启动时加载了关节映射文件")
    
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

