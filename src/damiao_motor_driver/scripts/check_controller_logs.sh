#!/bin/bash
# 检查控制器相关的日志

echo "=========================================="
echo "检查控制器启动日志"
echo "=========================================="

echo ""
echo "1. 检查 controller_spawner 节点状态..."
if rosnode list 2>/dev/null | grep -q controller_spawner; then
    echo "   ✓ controller_spawner 节点存在"
    echo "   节点信息:"
    timeout 3 rosnode info /motor_hw_interface/controller_spawner 2>&1 | head -10
else
    echo "   ✗ controller_spawner 节点不存在"
fi

echo ""
echo "2. 检查 joint_state_controller 相关话题..."
JOINT_STATES_TOPICS=$(rostopic list 2>/dev/null | grep joint_states)
if [ -n "$JOINT_STATES_TOPICS" ]; then
    echo "   ✓ 找到 joint_states 话题:"
    echo "$JOINT_STATES_TOPICS" | sed 's/^/     /'
    
    echo ""
    echo "   检查 /motor_hw_interface/joint_states 是否有数据..."
    if timeout 3 rostopic echo /motor_hw_interface/joint_states -n 1 > /dev/null 2>&1; then
        echo "   ✓ 有数据"
        echo "   数据示例:"
        timeout 3 rostopic echo /motor_hw_interface/joint_states -n 1 | head -15
    else
        echo "   ✗ 无数据"
    fi
else
    echo "   ✗ 未找到 joint_states 话题"
fi

echo ""
echo "3. 检查 rosout 日志中的关键信息..."
echo "   搜索 'controller' 相关日志（最近20条）..."
timeout 5 rostopic echo /rosout -n 20 2>/dev/null | \
    grep -E "(controller|joint_state|ERROR|WARN|spawner)" | \
    tail -10 || echo "   无法获取日志或超时"

echo ""
echo "4. 检查硬件接口节点状态..."
if rosnode list 2>/dev/null | grep -q "motor_hw_interface"; then
    echo "   ✓ motor_hw_interface 节点存在"
    echo "   检查发布的话题:"
    timeout 3 rosnode info /motor_hw_interface/motor_hw_interface 2>&1 | grep -A 10 "Publications" | head -10
else
    echo "   ✗ motor_hw_interface 节点不存在"
fi

echo ""
echo "=========================================="
echo "建议:"
echo "  如果看不到日志，请重新启动 launch 文件并查看终端输出"
echo "  或者使用: rosrun rqt_console rqt_console"
echo "=========================================="

