#!/bin/bash
# 快速检查 joint_state_controller 状态

echo "=========================================="
echo "快速检查 joint_state_controller"
echo "=========================================="

echo ""
echo "1. 检查话题发布者..."
rostopic info /motor_hw_interface/joint_states 2>&1 | grep -A 5 "Publishers"

echo ""
echo "2. 检查是否有数据（3秒超时）..."
timeout 3 rostopic echo /motor_hw_interface/joint_states -n 1 2>&1 | head -10

echo ""
echo "3. 检查 rosout 日志中的控制器信息..."
timeout 3 rostopic echo /rosout -n 30 2>&1 | grep -E "(joint_state_controller|Started|Loading|ERROR)" | tail -10

echo ""
echo "=========================================="
echo "关键发现:"
echo "  - 如果发布者是 motor_hw_interface 节点，说明 joint_state_controller 未运行"
echo "  - 如果发布者是 joint_state_controller，说明控制器运行但无数据"
echo "  - 检查启动日志中是否有 'Started controller: joint_state_controller'"
echo "=========================================="

