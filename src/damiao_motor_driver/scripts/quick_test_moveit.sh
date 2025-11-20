#!/bin/bash
# MoveIt 快速测试脚本

echo "=========================================="
echo "MoveIt 系统状态检查"
echo "=========================================="

echo ""
echo "1. 检查 joint_states 话题..."
# 先检查原始话题
if timeout 2 rostopic echo /motor_hw_interface/joint_states -n 1 > /dev/null 2>&1; then
    echo "   ✓ /motor_hw_interface/joint_states 话题正常"
    echo "   数据示例:"
    timeout 2 rostopic echo /motor_hw_interface/joint_states -n 1 | head -10
else
    echo "   ✗ /motor_hw_interface/joint_states 话题无数据"
    echo "   可能原因: joint_state_controller 未启动或硬件接口无反馈"
fi

# 检查重映射后的话题
if timeout 2 rostopic echo /joint_states -n 1 > /dev/null 2>&1; then
    echo "   ✓ /joint_states 话题正常（重映射）"
else
    echo "   ✗ /joint_states 话题无数据（重映射失败）"
    echo "   检查: rostopic list | grep joint_states"
fi

echo ""
echo "2. 检查 move_group 服务..."
rosservice list | grep -q "/move_group"
if [ $? -eq 0 ]; then
    echo "   ✓ move_group 服务可用"
    echo "   可用服务:"
    rosservice list | grep move_group | head -5
else
    echo "   ✗ move_group 服务不可用"
fi

echo ""
echo "3. 检查控制器状态..."
if timeout 5 rosservice call /motor_hw_interface/controller_manager/list_controllers 2>/dev/null | head -30; then
    echo "   ✓ 控制器状态获取成功"
else
    echo "   ✗ 控制器状态获取失败或超时"
    echo "   检查服务是否存在:"
    rosservice list | grep controller_manager | head -5
fi

echo ""
echo "4. 检查规划组..."
rosservice call /move_group/get_planning_scene "components:
  components: 0" 2>/dev/null | grep -q "robot_state"
if [ $? -eq 0 ]; then
    echo "   ✓ MoveIt 规划场景正常"
else
    echo "   ✗ MoveIt 规划场景异常"
fi

echo ""
echo "=========================================="
echo "运行 Python 测试脚本:"
echo "python3 src/damiao_motor_driver/scripts/test_moveit_execution.py --group left_arm"
echo "=========================================="

