#!/bin/bash
# 手动启动 joint_state_controller

echo "=========================================="
echo "手动启动 joint_state_controller"
echo "=========================================="

echo ""
echo "步骤 1: 检查控制器是否已加载..."
timeout 3 rosservice call /motor_hw_interface/controller_manager/list_controllers 2>&1 | grep -A 10 "joint_state_controller" || echo "无法检查（可能服务调用超时）"

echo ""
echo "步骤 2: 尝试加载控制器..."
echo "执行: rosservice call /motor_hw_interface/controller_manager/load_controller 'joint_state_controller'"
timeout 5 rosservice call /motor_hw_interface/controller_manager/load_controller "joint_state_controller" 2>&1

echo ""
echo "步骤 3: 尝试启动控制器..."
echo "执行: rosservice call /motor_hw_interface/controller_manager/switch_controller ..."
timeout 5 rosservice call /motor_hw_interface/controller_manager/switch_controller \
  "{start_controllers: ['joint_state_controller'], stop_controllers: [], strictness: 2, start_asap: false, timeout: 0.0}" 2>&1

echo ""
echo "步骤 4: 检查控制器状态..."
timeout 3 rosservice call /motor_hw_interface/controller_manager/list_controllers 2>&1 | grep -A 5 "joint_state_controller" || echo "无法检查（可能服务调用超时）"

echo ""
echo "步骤 5: 检查 joint_states 话题..."
timeout 3 rostopic echo /motor_hw_interface/joint_states -n 1 2>&1 | head -10

echo ""
echo "=========================================="
echo "如果控制器启动成功，你应该看到:"
echo "  - 控制器状态为 'running'"
echo "  - /motor_hw_interface/joint_states 有数据"
echo "=========================================="
