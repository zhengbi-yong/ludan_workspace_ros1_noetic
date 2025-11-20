#!/bin/bash
# 查看启动日志的脚本

echo "=========================================="
echo "查看启动日志的方法"
echo "=========================================="

echo ""
echo "方法1: 查看当前运行的节点的日志"
echo "----------------------------------------"
echo "如果 launch 文件还在运行，日志就在启动它的终端窗口中"
echo ""
echo "如果已经关闭，可以查看 rosout 日志:"
echo "  rosrun rqt_console rqt_console"
echo "  或者:"
echo "  rostopic echo /rosout | grep -E '(controller|joint_state|motor_hw)'"
echo ""

echo "方法2: 重新启动并保存日志到文件"
echo "----------------------------------------"
echo "roslaunch damiao_motor_driver moveit_with_motor_driver.launch \\"
echo "  port:=/dev/ttyACM0 \\"
echo "  baud:=921600 \\"
echo "  joint_mapping:=\$(find damiao_motor_driver)/config/joint_mapping_example.yaml \\"
echo "  2>&1 | tee /tmp/moveit_startup.log"
echo ""
echo "然后查看日志文件:"
echo "  cat /tmp/moveit_startup.log | grep -E '(controller|joint_state|motor_hw|ERROR|WARN)'"
echo ""

echo "方法3: 查看特定节点的日志"
echo "----------------------------------------"
echo "检查节点是否运行:"
echo "  rosnode list | grep -E '(controller|motor_hw)'"
echo ""
echo "查看节点信息:"
echo "  rosnode info /motor_hw_interface/controller_spawner"
echo "  rosnode info /motor_hw_interface/motor_hw_interface"
echo ""

echo "方法4: 使用 rqt 图形界面查看日志"
echo "----------------------------------------"
echo "  rosrun rqt_console rqt_console"
echo "  在界面中可以过滤和搜索日志"
echo ""

echo "=========================================="
echo "快速检查关键信息"
echo "=========================================="
echo ""
echo "检查 controller_spawner 节点:"
rosnode list 2>/dev/null | grep -q controller_spawner && echo "  ✓ controller_spawner 节点存在" || echo "  ✗ controller_spawner 节点不存在"

echo ""
echo "检查 joint_state_controller 相关话题:"
rostopic list 2>/dev/null | grep joint_states | head -3

echo ""
echo "检查是否有错误日志（最近10条）:"
timeout 2 rostopic echo /rosout -n 10 2>/dev/null | grep -E "(ERROR|WARN|controller|joint_state)" | tail -5 || echo "  无法获取日志"

echo ""
echo "=========================================="

