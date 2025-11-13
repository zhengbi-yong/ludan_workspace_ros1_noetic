cmake_minimum_required(VERSION 3.10)
project(legged_dm_hw)

# ========================
# 🔧 编译设置
# ========================
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================
# 🔍 查找依赖
# ========================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  legged_hw
  tf
  serial
)

find_package(Boost REQUIRED)

# ========================
# 🔧 手动补全库路径（关键修复点）
# ========================
link_directories(/opt/ros/noetic/lib)

# ========================
# 📦 catkin 配置
# ========================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp legged_hw tf serial
)

# ========================
# 📁 头文件目录
# ========================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# ========================
# ⚙️ 可执行文件
# ========================
add_library(${PROJECT_NAME}
  src/legged_dm_hw.cpp
  src/DmHW.cpp
)

# ========================
# 🔗 链接库（修复 DSO missing & not found）
# ========================
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  roscpp
  rosconsole
  rostime
  pthread
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# ========================
# 🧩 clang 工具（保持）
# ========================
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ${PROJECT_NAME}")
    add_clang_tooling(
        TARGETS ${PROJECT_NAME}
        SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
        CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CF_WERROR
    )
endif()

# ========================
# 📦 安装
# ========================
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
