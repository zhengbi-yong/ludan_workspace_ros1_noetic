cmake_minimum_required(VERSION 3.0.2)
project(damiao_motor_driver)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    diagnostic_updater
    hardware_interface
    controller_manager
    pluginlib
    damiao_motor_control_board_serial
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motor_driver motor_hw_interface
    CATKIN_DEPENDS roscpp serial diagnostic_updater hardware_interface controller_manager pluginlib
)

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
    motor_driver
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

install(TARGETS motor_driver motor_hw_interface motor_driver_node
                send_single_motor_node trajectory_controller_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

