#!/usr/bin/env python3
"""
检查控制器节点状态（不调用服务，避免卡住）
"""

import rospy
import sys

def check_nodes():
    """检查相关节点"""
    rospy.init_node('check_controller_nodes', anonymous=True)
    
    print("=" * 60)
    print("节点状态检查")
    print("=" * 60)
    
    try:
        import rosnode
        nodes = rosnode.get_node_names()
        
        print("\n相关节点:")
        motor_hw_found = False
        controller_spawner_found = False
        joint_states_relay_found = False
        
        for node in nodes:
            if 'motor_hw_interface' in node and 'motor_hw_interface' == node.split('/')[-1]:
                print(f"  ✓ {node}")
                motor_hw_found = True
            elif 'controller_spawner' in node:
                print(f"  ✓ {node}")
                controller_spawner_found = True
            elif 'joint_states_relay' in node:
                print(f"  ✓ {node}")
                joint_states_relay_found = True
        
        if not motor_hw_found:
            print("  ✗ motor_hw_interface 节点未找到")
        if not controller_spawner_found:
            print("  ✗ controller_spawner 节点未找到")
        if not joint_states_relay_found:
            print("  ✗ joint_states_relay 节点未找到")
        
        # 检查节点是否响应
        print("\n节点响应检查:")
        if motor_hw_found:
            try:
                node_info = rosnode.get_node_info('/motor_hw_interface/motor_hw_interface', timeout=2.0)
                print("  ✓ motor_hw_interface 节点响应正常")
            except:
                print("  ✗ motor_hw_interface 节点无响应")
        
        return motor_hw_found and controller_spawner_found
        
    except Exception as e:
        print(f"检查失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    try:
        success = check_nodes()
        print("\n" + "=" * 60)
        if success:
            print("节点检查完成")
        else:
            print("部分节点缺失")
        print("=" * 60)
    except KeyboardInterrupt:
        print("\n中断")
    except Exception as e:
        print(f"\n错误: {str(e)}")


if __name__ == '__main__':
    main()

