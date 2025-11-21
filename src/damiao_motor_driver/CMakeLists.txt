cmake_minimum_required(VERSION 3.0.2)
project(damiao_motor_driver)

add_compile_options(-std=c++17 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    diagnostic_updater
    dynamic_reconfigure
    hardware_interface
    controller_manager
    pluginlib
    std_msgs
    geometry_msgs
    rosbag
    message_generation
    damiao_motor_control_board_serial
    damiao_motor_core
)

generate_dynamic_reconfigure_options(
    cfg/driver_limits.cfg
)
add_message_files(
    FILES
    MotorCommand.msg           # 你刚刚新建的消息
    # 以后有其他 msg 也可以加在这里
)

add_service_files(
    FILES
    SendCommand.srv
    SendCommands.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motor_hw_interface
    CATKIN_DEPENDS roscpp rospy serial diagnostic_updater dynamic_reconfigure
        hardware_interface controller_manager pluginlib std_msgs geometry_msgs rosbag
        message_runtime damiao_motor_core
)

# Export the hardware_interface plugin description so pluginlib can discover
# MotorHWInterface at runtime. Use the modern macro if available, otherwise
# manually copy the file (silently, without warning).
set(_motor_hw_plugin_xml ${CMAKE_CURRENT_SOURCE_DIR}/motor_hw_interface_plugin.xml)
if(COMMAND pluginlib_export_plugin_description_file)
    pluginlib_export_plugin_description_file(
        hardware_interface
        ${_motor_hw_plugin_xml}
    )
else()
    # Silently copy plugin file for older ROS distributions
    set(_motor_hw_plugin_dest
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/motor_hw_interface_plugin.xml
    )
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
    configure_file(${_motor_hw_plugin_xml} ${_motor_hw_plugin_dest} COPYONLY)
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# 底层库（串口 + 协议 + 驱动）
add_library(motor_driver
    src/motor_serial.cpp
    # src/motor_protocol.cpp
    src/motor_driver.cpp
)

target_link_libraries(motor_driver
    ${catkin_LIBRARIES}
)


# ros_control 插件
add_library(motor_hw_interface
    src/motor_hw_interface.cpp
)
target_link_libraries(motor_hw_interface
    ${catkin_LIBRARIES}
    ${damiao_motor_core_LIBRARIES}
)


# ros_control 硬件接口节点
add_executable(motor_hw_interface_node src/motor_hw_interface_node.cpp)
target_link_libraries(motor_hw_interface_node
    motor_hw_interface
    ${catkin_LIBRARIES}
)

# 启动驱动节点（唯一占用串口）- 使用新的MotorDriverNode
add_executable(motor_driver_node src/motor_driver_node_main.cpp src/motor_driver_node.cpp)
target_link_libraries(motor_driver_node
    ${catkin_LIBRARIES}
    ${damiao_motor_core_LIBRARIES}
)
add_dependencies(motor_driver_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# 话题控制桥接节点：订阅 /motor_cmd -> 调用 ROS 服务
add_executable(motor_cmd_bridge_node src/motor_cmd_bridge.cpp)
target_link_libraries(motor_cmd_bridge_node
    ${catkin_LIBRARIES}
)
add_dependencies(motor_cmd_bridge_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# 测试节点（使用ROS服务接口）
add_executable(send_single_motor_node test_nodes/send_single_motor_node.cpp)
target_link_libraries(send_single_motor_node
    ${catkin_LIBRARIES}
)
add_dependencies(send_single_motor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(trajectory_controller_node test_nodes/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node
    ${catkin_LIBRARIES}
)
add_dependencies(trajectory_controller_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
add_dependencies(motor_cmd_bridge_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(motor_driver
    ${PROJECT_NAME}_gencfg
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
add_dependencies(motor_hw_interface
    ${PROJECT_NAME}_gencfg
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(motor_driver_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(motor_protocol_test test/protocol_test.cpp)
    target_link_libraries(motor_protocol_test motor_driver ${catkin_LIBRARIES})

    catkin_add_gtest(motor_driver_integration_test test/driver_integration_test.cpp)
    target_link_libraries(motor_driver_integration_test motor_driver ${catkin_LIBRARIES})

    catkin_add_gtest(motor_hw_interface_test test/motor_hw_interface_test.cpp)
    target_link_libraries(motor_hw_interface_test motor_hw_interface ${catkin_LIBRARIES})
endif()

install(TARGETS motor_driver motor_hw_interface motor_driver_node
                motor_hw_interface_node
                send_single_motor_node trajectory_controller_node
                motor_cmd_bridge_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
        motor_hw_interface_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# policy_bridge.py has been moved to damiao_policy_bridge package

