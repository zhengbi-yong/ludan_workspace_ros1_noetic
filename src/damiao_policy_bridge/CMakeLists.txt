cmake_minimum_required(VERSION 3.0.2)
project(damiao_policy_bridge)

# 确保能够找到ROS包
# 如果CMAKE_PREFIX_PATH不包含/opt/ros/noetic，手动添加
if(CMAKE_PREFIX_PATH)
    list(FIND CMAKE_PREFIX_PATH "/opt/ros/noetic" _index)
    if(_index EQUAL -1)
        list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
    endif()
else()
    set(CMAKE_PREFIX_PATH "/opt/ros/noetic")
endif()

# 确保能够找到damiao_motor_control_board_serial
# 从CMAKE_PREFIX_PATH中查找配置文件（包括符号链接）
if(NOT damiao_motor_control_board_serial_DIR)
    foreach(_prefix ${CMAKE_PREFIX_PATH})
        set(_config_file "${_prefix}/share/damiao_motor_control_board_serial/cmake/damiao_motor_control_board_serialConfig.cmake")
        if(EXISTS "${_config_file}" OR IS_SYMLINK "${_config_file}")
            get_filename_component(damiao_motor_control_board_serial_DIR "${_config_file}" DIRECTORY)
            set(damiao_motor_control_board_serial_DIR "${damiao_motor_control_board_serial_DIR}" CACHE PATH "")
            break()
        endif()
    endforeach()
endif()

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    rosbag
    damiao_motor_control_board_serial
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs geometry_msgs rosbag damiao_motor_control_board_serial
)

catkin_install_python(PROGRAMS
    nodes/policy_bridge_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY lib/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

