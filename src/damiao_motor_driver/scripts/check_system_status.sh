#!/bin/bash
# 检查系统状态，确认所有必要的组件是否都启动了

echo "=========================================="
echo "检查系统状态"
echo "=========================================="

echo ""
echo "1. 检查核心节点..."
echo "----------------------------------------"

# 检查硬件接口节点
if rosnode list 2>/dev/null | grep -q "motor_hw_interface"; then
    echo "   ✓ motor_hw_interface 节点运行中"
else
    echo "   ✗ motor_hw_interface 节点未运行"
fi

# 检查 controller_spawner
if rosnode list 2>/dev/null | grep -q "controller_spawner"; then
    echo "   ✓ controller_spawner 节点运行中"
else
    echo "   ✗ controller_spawner 节点未运行"
fi

# 检查 move_group
if rosnode list 2>/dev/null | grep -q "move_group"; then
    echo "   ✓ move_group 节点运行中"
else
    echo "   ✗ move_group 节点未运行"
fi

# 检查 robot_state_publisher
if rosnode list 2>/dev/null | grep -q "robot_state_publisher"; then
    echo "   ✓ robot_state_publisher 节点运行中"
else
    echo "   ✗ robot_state_publisher 节点未运行"
fi

# 检查 joint_states_relay
if rosnode list 2>/dev/null | grep -q "joint_states_relay"; then
    echo "   ✓ joint_states_relay 节点运行中"
else
    echo "   ✗ joint_states_relay 节点未运行"
fi

echo ""
echo "2. 检查关键话题..."
echo "----------------------------------------"

# 检查 joint_states
if timeout 2 rostopic list 2>/dev/null | grep -q "/joint_states"; then
    echo "   ✓ /joint_states 话题存在"
    if timeout 3 rostopic echo /joint_states -n 1 > /dev/null 2>&1; then
        echo "   ✓ /joint_states 有数据"
    else
        echo "   ✗ /joint_states 无数据"
    fi
else
    echo "   ✗ /joint_states 话题不存在"
fi

# 检查 motor_states
if timeout 2 rostopic list 2>/dev/null | grep -q "motor_states"; then
    echo "   ✓ motor_states 话题存在"
    if timeout 3 rostopic echo /motor_hw_interface/motor_hw_interface/motor_states -n 1 > /dev/null 2>&1; then
        echo "   ✓ motor_states 有数据"
    else
        echo "   ✗ motor_states 无数据"
    fi
else
    echo "   ✗ motor_states 话题不存在"
fi

echo ""
echo "3. 检查 MoveIt 服务..."
echo "----------------------------------------"

# 检查 move_group 服务
if timeout 2 rosservice list 2>/dev/null | grep -q "/move_group"; then
    echo "   ✓ move_group 服务可用"
    echo "   可用服务示例:"
    timeout 2 rosservice list 2>/dev/null | grep move_group | head -3 | sed 's/^/     /'
else
    echo "   ✗ move_group 服务不可用"
fi

echo ""
echo "4. 检查控制器管理器..."
echo "----------------------------------------"

# 检查 controller_manager 服务
if timeout 2 rosservice list 2>/dev/null | grep -q "controller_manager"; then
    echo "   ✓ controller_manager 服务可用"
else
    echo "   ✗ controller_manager 服务不可用"
fi

echo ""
echo "=========================================="
echo "总结"
echo "=========================================="
echo ""
echo "moveit_with_motor_driver.launch 应该启动:"
echo "  1. motor_hw_interface_node - 硬件接口"
echo "  2. controller_spawner - 控制器管理器"
echo "  3. joint_states_relay - 关节状态转发"
echo "  4. robot_state_publisher - TF 发布"
echo "  5. move_group - MoveIt 核心节点"
echo ""
echo "可选组件:"
echo "  - RViz (use_rviz:=true 时启动)"
echo ""
echo "如果所有组件都运行正常，你只需要启动这一个 launch 文件即可。"
echo "=========================================="

