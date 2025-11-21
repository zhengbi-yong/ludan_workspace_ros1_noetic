cmake_minimum_required(VERSION 3.0.2)
project(damiao_policy_bridge)

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

