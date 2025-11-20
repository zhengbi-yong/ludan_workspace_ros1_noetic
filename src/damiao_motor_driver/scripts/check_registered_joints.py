#!/usr/bin/env python3
"""
检查硬件接口实际注册的关节名称
"""

import rospy
import sys

def check_registered_joints():
    """检查硬件接口注册的关节"""
    rospy.init_node('check_registered_joints', anonymous=True)
    
    print("=" * 60)
    print("检查硬件接口注册的关节")
    print("=" * 60)
    
    # 等待参数服务器就绪
    rospy.sleep(0.5)
    
    # 检查关节配置
    print("\n1. 检查关节配置参数...")
    try:
        # 检查节点的私有命名空间
        joints_private = rospy.get_param('/motor_hw_interface/motor_hw_interface/joints', None)
        if joints_private:
            print(f"   ✓ 找到节点私有命名空间的关节配置: {len(joints_private)} 个关节")
            print("   前5个关节:")
            for i, joint in enumerate(joints_private[:5]):
                if isinstance(joint, dict):
                    name = joint.get('name', 'unknown')
                    motor_id = joint.get('id', -1)
                    print(f"     {name} -> 电机ID {motor_id}")
        else:
            print("   ✗ 节点私有命名空间无关节配置")
        
        # 检查组命名空间
        joints_group = rospy.get_param('/motor_hw_interface/joints', None)
        if joints_group:
            print(f"   ✓ 找到组命名空间的关节配置: {len(joints_group)} 个关节")
        else:
            print("   ✗ 组命名空间无关节配置")
        
        # 检查 tf_prefix
        tf_prefix = rospy.get_param('/motor_hw_interface/motor_hw_interface/tf_prefix', '')
        print(f"\n2. 检查 tf_prefix: '{tf_prefix}'")
        if tf_prefix:
            print(f"   ⚠ 注意: tf_prefix 不为空，关节名称会有前缀")
            print(f"   实际注册的关节名称格式: {tf_prefix}_<关节名>")
        else:
            print("   ✓ tf_prefix 为空，关节名称无前缀")
        
        # 检查控制器配置
        print("\n3. 检查控制器配置...")
        controller_joints = rospy.get_param('/motor_hw_interface/joint_state_controller/joints', None)
        if controller_joints:
            print(f"   ✓ 控制器配置的关节列表: {len(controller_joints)} 个关节")
            print("   前5个关节:")
            for name in controller_joints[:5]:
                print(f"     {name}")
        else:
            print("   ✗ 控制器配置无关节列表")
        
        # 比较关节名称
        print("\n4. 比较关节名称...")
        if joints_private and controller_joints:
            print("   检查关节名称是否匹配...")
            mismatch_count = 0
            for i, joint in enumerate(joints_private):
                if isinstance(joint, dict):
                    config_name = joint.get('name', '')
                    # 计算实际注册的名称（考虑 tf_prefix）
                    actual_name = tf_prefix + "_" + config_name if tf_prefix else config_name
                    
                    if actual_name not in controller_joints:
                        mismatch_count += 1
                        if mismatch_count <= 5:
                            print(f"     ✗ 不匹配: 配置={config_name}, 实际={actual_name}, 控制器期望={controller_joints[i] if i < len(controller_joints) else 'N/A'}")
            
            if mismatch_count == 0:
                print("   ✓ 所有关节名称匹配")
            else:
                print(f"   ✗ 有 {mismatch_count} 个关节名称不匹配")
                print("   解决方案: 确保控制器配置中的关节名称与硬件接口注册的名称一致")
        
    except Exception as e:
        print(f"   ✗ 检查失败: {str(e)}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)


if __name__ == '__main__':
    try:
        check_registered_joints()
    except KeyboardInterrupt:
        print("\n中断")
    except Exception as e:
        print(f"\n错误: {str(e)}")
        import traceback
        traceback.print_exc()

