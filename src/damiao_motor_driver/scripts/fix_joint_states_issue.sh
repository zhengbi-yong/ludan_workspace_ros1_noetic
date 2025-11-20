#!/bin/bash
# 修复 joint_states 问题的综合脚本

echo "=========================================="
echo "Joint States 问题诊断和修复"
echo "=========================================="

echo ""
echo "步骤 1: 检查关节映射配置..."
if rosparam list | grep -q "/motor_hw_interface/joints"; then
    echo "   ✓ 关节映射已加载"
    JOINT_COUNT=$(rosparam get /motor_hw_interface/joints 2>/dev/null | grep -c "name:" || echo "0")
    echo "   关节数量: $JOINT_COUNT"
else
    echo "   ✗ 关节映射未加载！"
    echo ""
    echo "   这是问题的根源！"
    echo "   解决方案: 重新启动时添加关节映射参数"
    echo ""
    echo "   正确启动命令:"
    echo "   roslaunch damiao_motor_driver moveit_with_motor_driver.launch \\"
    echo "     port:=/dev/ttyACM0 \\"
    echo "     baud:=921600 \\"
    echo "     joint_mapping:=\$(find damiao_motor_driver)/config/joint_mapping_example.yaml"
    echo ""
    exit 1
fi

echo ""
echo "步骤 2: 检查电机反馈..."
if timeout 2 rostopic echo /motor_hw_interface/motor_states -n 1 > /dev/null 2>&1; then
    echo "   ✓ 电机反馈正常"
else
    echo "   ✗ 无电机反馈"
    echo "   检查: 硬件连接、串口权限、电机是否上电"
fi

echo ""
echo "步骤 3: 检查节点状态..."
if rosnode list | grep -q "controller_spawner"; then
    echo "   ✓ controller_spawner 节点存在"
else
    echo "   ✗ controller_spawner 节点不存在"
fi

if rosnode list | grep -q "joint_states_relay"; then
    echo "   ✓ joint_states_relay 节点存在"
else
    echo "   ✗ joint_states_relay 节点不存在"
fi

echo ""
echo "步骤 4: 检查话题..."
if timeout 2 rostopic echo /motor_hw_interface/joint_states -n 1 > /dev/null 2>&1; then
    echo "   ✓ /motor_hw_interface/joint_states 有数据"
    echo "   数据示例:"
    timeout 2 rostopic echo /motor_hw_interface/joint_states -n 1 | head -10
else
    echo "   ✗ /motor_hw_interface/joint_states 无数据"
    echo ""
    echo "   可能原因:"
    echo "   1. joint_state_controller 未运行"
    echo "   2. 硬件接口没有关节数据（关节映射未加载）"
    echo "   3. 电机反馈异常"
fi

echo ""
echo "=========================================="
echo "建议的修复步骤:"
echo "=========================================="
echo ""
echo "如果关节映射未加载，请:"
echo "1. 停止当前启动的节点 (Ctrl+C)"
echo "2. 使用正确的启动命令（包含 joint_mapping 参数）"
echo ""
echo "如果关节映射已加载但仍无数据，请:"
echo "1. 运行: rosrun damiao_motor_driver simple_check_joint_states.py"
echo "2. 检查硬件连接和电机反馈"
echo "3. 查看启动日志中的错误信息"
echo ""

