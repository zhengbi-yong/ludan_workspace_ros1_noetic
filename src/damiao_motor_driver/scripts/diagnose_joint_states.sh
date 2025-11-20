#!/bin/bash
# 诊断 joint_states 问题

echo "=========================================="
echo "Joint States 诊断工具"
echo "=========================================="

echo ""
echo "1. 检查话题列表..."
echo "所有 joint_states 相关话题:"
rostopic list | grep -i joint

echo ""
echo "2. 检查 joint_state_controller 状态..."
if rosservice list | grep -q "controller_manager/list_controllers"; then
    echo "   尝试获取控制器列表（使用 Python 脚本，避免卡住）..."
    timeout 10 python3 -c "
import rospy
from controller_manager_msgs.srv import ListControllers
rospy.init_node('check_controllers', anonymous=True)
try:
    list_controllers = rospy.ServiceProxy('/motor_hw_interface/controller_manager/list_controllers', ListControllers)
    rospy.wait_for_service('/motor_hw_interface/controller_manager/list_controllers', timeout=3.0)
    response = list_controllers()
    print('   控制器列表:')
    for c in response.controller:
        print(f'     - {c.name}: {c.state}')
except Exception as e:
    print(f'   错误: {e}')
" 2>&1
else
    echo "   ✗ controller_manager 服务不可用"
fi

echo ""
echo "3. 检查原始话题 /motor_hw_interface/joint_states..."
if rostopic list | grep -q "/motor_hw_interface/joint_states"; then
    echo "   ✓ 话题存在"
    echo "   检查是否有发布者:"
    rostopic info /motor_hw_interface/joint_states | grep -A 5 "Publishers"
    echo "   尝试接收一条消息（3秒超时）:"
    timeout 3 rostopic echo /motor_hw_interface/joint_states -n 1 2>&1 | head -15
else
    echo "   ✗ 话题不存在"
fi

echo ""
echo "4. 检查重映射话题 /joint_states..."
if rostopic list | grep -q "^/joint_states$"; then
    echo "   ✓ 话题存在"
    echo "   检查是否有发布者:"
    rostopic info /joint_states | grep -A 5 "Publishers"
    echo "   尝试接收一条消息（3秒超时）:"
    timeout 3 rostopic echo /joint_states -n 1 2>&1 | head -15
else
    echo "   ✗ 话题不存在"
fi

echo ""
echo "5. 检查节点状态..."
echo "   motor_hw_interface 相关节点:"
rosnode list | grep -E "(motor_hw_interface|joint_states|controller)"

echo ""
echo "6. 检查硬件接口反馈..."
if rostopic list | grep -q "/motor_hw_interface/motor_states"; then
    echo "   检查电机反馈数据:"
    timeout 2 rostopic echo /motor_hw_interface/motor_states -n 1 2>&1 | head -10
else
    echo "   ✗ 电机反馈话题不存在"
fi

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="

