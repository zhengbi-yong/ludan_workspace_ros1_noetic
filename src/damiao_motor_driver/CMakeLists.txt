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
    damiao_motor_control_board_serial
)

generate_dynamic_reconfigure_options(
    cfg/driver_limits.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motor_driver motor_hw_interface
    CATKIN_DEPENDS roscpp rospy serial diagnostic_updater dynamic_reconfigure
        hardware_interface controller_manager pluginlib std_msgs geometry_msgs rosbag
)

# Ensure the hardware_interface plugin description is available in both the
# devel and install spaces so pluginlib can discover the MotorHWInterface at
# runtime.
set(_motor_hw_plugin_xml ${CMAKE_CURRENT_SOURCE_DIR}/motor_hw_interface_plugin.xml)
set(_motor_hw_plugin_dest
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/motor_hw_interface_plugin.xml
)

# Older ROS installations might not ship the pluginlib CMake helper macro
# (pluginlib_export_plugin_description_file).  Copy the XML into the devel space
# manually so that the <export> tag in package.xml still resolves correctly.
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
configure_file(${_motor_hw_plugin_xml} ${_motor_hw_plugin_dest} COPYONLY)

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
add_dependencies(motor_driver ${PROJECT_NAME}_gencfg)

# ros_control 插件
add_library(motor_hw_interface
    src/motor_hw_interface.cpp
)
target_link_libraries(motor_hw_interface
    motor_driver
    ${catkin_LIBRARIES}
)
add_dependencies(motor_hw_interface ${PROJECT_NAME}_gencfg)

# ros_control 硬件接口节点
add_executable(motor_hw_interface_node src/motor_hw_interface_node.cpp)
target_link_libraries(motor_hw_interface_node
    motor_hw_interface
    ${catkin_LIBRARIES}
)

# 启动驱动节点（唯一占用串口）
add_executable(motor_driver_node src/motor_driver_node.cpp)
target_link_libraries(motor_driver_node
    motor_driver
    ${catkin_LIBRARIES}
)

# 单独控制任意电机
add_executable(send_single_motor_node src/send_single_motor_node.cpp)
target_link_libraries(send_single_motor_node
    motor_driver
    ${catkin_LIBRARIES}
)

# 轨迹跟踪控制器
add_executable(trajectory_controller_node src/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node
    motor_driver
    ${catkin_LIBRARIES}
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
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
        motor_hw_interface_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/policy_bridge.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

