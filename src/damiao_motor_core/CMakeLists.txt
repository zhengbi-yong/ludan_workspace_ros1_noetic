cmake_minimum_required(VERSION 3.0.2)
project(damiao_motor_core)

# 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项（添加-fPIC以支持链接到共享库）
add_compile_options(-Wall -Wextra -O3 -fPIC)

# 查找依赖（不依赖ROS，只依赖serial库用于串口）
find_package(catkin REQUIRED)
find_package(serial REQUIRED)

# 包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${serial_INCLUDE_DIRS}
)

# 核心库源文件
set(CORE_SOURCES
    src/transport.cpp
    src/serial_transport.cpp
    src/protocol.cpp
    src/motor_state.cpp
    src/motor_command.cpp
    src/ring_buffer.cpp
    src/command_queue.cpp
    src/error_handler.cpp
    src/config.cpp
    src/motor_driver_core.cpp
)

# 创建静态库（需要-fPIC以支持链接到共享库）
add_library(damiao_motor_core STATIC ${CORE_SOURCES})
set_target_properties(damiao_motor_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(damiao_motor_core
    ${serial_LIBRARIES}
)

# catkin_package必须在install之前
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES damiao_motor_core
    CATKIN_DEPENDS
    DEPENDS serial
)

# 安装规则
install(TARGETS damiao_motor_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

# 单元测试
if(CATKIN_ENABLE_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_protocol test/test_protocol.cpp)
    target_link_libraries(test_protocol damiao_motor_core ${GTEST_LIBRARIES} pthread)
    add_test(NAME test_protocol COMMAND test_protocol)
    
    add_executable(test_ring_buffer test/test_ring_buffer.cpp)
    target_link_libraries(test_ring_buffer damiao_motor_core ${GTEST_LIBRARIES} pthread)
    add_test(NAME test_ring_buffer COMMAND test_ring_buffer)
    
    add_executable(test_command_queue test/test_command_queue.cpp)
    target_link_libraries(test_command_queue damiao_motor_core ${GTEST_LIBRARIES} pthread)
    add_test(NAME test_command_queue COMMAND test_command_queue)
    
    add_executable(test_motor_driver_core test/test_motor_driver_core.cpp)
    target_link_libraries(test_motor_driver_core damiao_motor_core ${GTEST_LIBRARIES} pthread)
    add_test(NAME test_motor_driver_core COMMAND test_motor_driver_core)
endif()

