#!/bin/bash
# 调试 joint_state_controller

echo "=========================================="
echo "调试 joint_state_controller"
echo "=========================================="

echo ""
echo "1. 检查硬件接口注册的关节..."
echo "   从参数服务器读取关节配置:"
rosparam get /motor_hw_interface/joints 2>/dev/null | head -20 || echo "   无法读取关节配置"

echo ""
echo "2. 检查 joint_state_controller 配置..."
rosparam get /motor_hw_interface/joint_state_controller 2>/dev/null || echo "   无法读取控制器配置"

echo ""
echo "3. 检查硬件接口是否发布了 joint_states..."
timeout 2 rostopic info /motor_hw_interface/joint_states 2>&1 | head -10

echo ""
echo "4. 尝试手动检查控制器状态（使用 Python 脚本避免卡住）..."
echo "   运行: python3 src/damiao_motor_driver/scripts/fix_joint_state_controller.py"

echo ""
echo "=========================================="
echo "关键问题:"
echo "  如果 joint_state_controller 加载了但没有启动，"
echo "  可能是因为硬件接口没有正确注册关节，或者"
echo "  控制器无法从硬件接口读取关节数据。"
echo ""
echo "  检查方法:"
echo "  1. 查看启动日志中是否有 'Started controller' 消息"
echo "  2. 检查硬件接口是否正确注册了关节"
echo "  3. 检查控制器配置是否正确"
echo "=========================================="

