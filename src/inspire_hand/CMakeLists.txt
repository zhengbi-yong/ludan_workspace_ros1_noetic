cmake_minimum_required(VERSION 3.0.2)
project(inspire_hand)

# ========================
# 🔧 编译配置
# ========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================
# 🔍 查找依赖
# ========================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
  tf
  std_msgs
  genmsg
)

# ========================
# 🔧 手动添加库路径（避免 DSO missing）
# ========================
link_directories(/opt/ros/noetic/lib)

# ========================
# 📦 消息 & 服务文件
# ========================
add_message_files(
  FILES
  get_force_act_1.msg
  get_angle_act_1.msg
  get_touch_act_1.msg
  set_angle__1.msg
  set_force_1.msg
  set_speed_1.msg
  hand_command.msg
)

set(SERVICE_FILES
  set_id.srv
  set_redu_ratio.srv
  set_clear_error.srv
  set_save_flash.srv
  set_reset_para.srv
  set_force_clb.srv
  set_gesture_no.srv
  set_current_limit.srv
  set_default_speed.srv
  set_default_force.srv
  set_user_def_angle.srv
  set_pos.srv
  set_angle.srv
  set_angle_1.srv
  set_force.srv
  set_speed.srv
  get_pos_act.srv
  get_angle_act.srv
  get_force_act.srv
  get_current.srv
  get_error.srv
  get_status.srv
  get_temp.srv
  get_pos_set.srv
  get_angle_set.srv
  get_force_set.srv
)

foreach(SERVICE_FILE ${SERVICE_FILES})
  add_service_files(FILES ${SERVICE_FILE})
endforeach()

generate_messages(DEPENDENCIES std_msgs)

# ========================
# 📦 catkin 包配置
# ========================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp serial tf std_msgs message_runtime
)

# ========================
# 📁 头文件 & 库目录
# ========================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========================
# ⚙️ 源文件和主可执行程序
# ========================
set(SOURCES
  src/hand_control.cpp
  src/hand_control_lib.cpp
)

set(HEADERS
  include/hand_control.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} inspire_hand_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  roscpp
  rosconsole
  rostime
  pthread
)

# ========================
# ⚙️ 其他可执行文件
# ========================
set(EXECUTABLES
  hand_control_client
  handcontroltopicpublisher
  handcontroltopicsubscriber
  handcontroltopicpublisher1
  handcontroltopicsubscriber1
)

foreach(EXECUTABLE ${EXECUTABLES})
  add_executable(${EXECUTABLE} src/${EXECUTABLE}.cpp)
  target_link_libraries(${EXECUTABLE}
    ${catkin_LIBRARIES}
    roscpp
    rosconsole
    rostime
    pthread
  )
  add_dependencies(${EXECUTABLE} inspire_hand_gencpp)
endforeach()

# ========================
# 🐍 Python 脚本安装
# ========================
catkin_install_python(PROGRAMS
  script/inspire_hand_serial_topic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ========================
# ⚙️ 输出路径设置（可选）
# ========================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
