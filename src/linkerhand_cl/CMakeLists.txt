cmake_minimum_required(VERSION 3.8)
project(linkerhand_cl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 包含目录
include_directories(include)

# 手部控制库源文件
set(HAND_CONTROL_SOURCES
  linkerhand_cl/hand_control.cpp
  linkerhand_cl/can_sender.cpp
  linkerhand_cl/can_receiver.cpp
)

# 手部控制库头文件
set(HAND_CONTROL_HEADERS
  include/linkerhand_cl/hand_control.hpp
  include/linkerhand_cl/can_sender.hpp
  include/linkerhand_cl/can_receiver.hpp
)

# 创建手部控制库
add_library(hand_control_lib ${HAND_CONTROL_SOURCES} ${HAND_CONTROL_HEADERS})
ament_target_dependencies(hand_control_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# ROS2节点可执行文件
add_executable(hand_control_node linkerhand_cl/hand_control_node.cpp)
ament_target_dependencies(hand_control_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(hand_control_node hand_control_lib)

# 安装
install(TARGETS
  hand_control_node
  hand_control_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

