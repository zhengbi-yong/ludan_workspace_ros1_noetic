# 启动日志分析

## 关键发现

从启动日志（Line 64-722）分析：

1. **Line 645**: `Loading controller: joint_state_controller` 
   - ✓ 控制器正在加载

2. **Line 669**: `Failed to read controllers from /controller_manager/list_controllers`
   - ✗ MoveIt 无法读取控制器列表（这可能是因为服务调用超时）

3. **缺少**: `Started controller: joint_state_controller`
   - ✗ 没有看到控制器启动成功的消息

## 问题诊断

**结论**: `joint_state_controller` 加载了但没有成功启动。

## 可能的原因

1. **硬件接口未完全初始化**: `controller_spawner` 在硬件接口完全初始化之前就尝试启动控制器
2. **控制器启动失败**: 控制器启动时出错，但没有显示错误信息
3. **服务调用超时**: `controller_manager` 服务响应慢，导致启动失败

## 解决方案

### 方案1: 增加延迟（已修改 launch 文件）

修改 `motor_hw_with_moveit.launch`，增加 `controller_spawner` 的启动延迟：

```xml
<node pkg="controller_manager" 
      type="spawner" 
      name="controller_spawner" 
      output="screen"
      launch-prefix="bash -c 'sleep 5; $0 $@'"
      args="--timeout 20 joint_state_controller" />
```

### 方案2: 手动启动控制器

如果方案1不起作用，可以手动启动：

```bash
# 加载控制器
rosservice call /motor_hw_interface/controller_manager/load_controller "joint_state_controller"

# 启动控制器
rosservice call /motor_hw_interface/controller_manager/switch_controller \
  "{start_controllers: ['joint_state_controller'], stop_controllers: [], strictness: 2}"
```

### 方案3: 使用 Python 脚本自动修复

运行修复脚本：

```bash
rosrun damiao_motor_driver check_and_fix_joint_states.py
```

## 验证

启动后，检查：

```bash
# 检查控制器状态
rosservice call /motor_hw_interface/controller_manager/list_controllers

# 检查 joint_states 话题
rostopic echo /motor_hw_interface/joint_states -n 1
```

