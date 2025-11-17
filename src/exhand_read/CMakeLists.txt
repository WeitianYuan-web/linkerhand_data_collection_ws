cmake_minimum_required(VERSION 3.8)
project(exhand_read)

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

# 串口通信库源文件
set(UART_FRAME_COMMAND_SOURCES
  exhand_read/uart_frame_command.cpp
)

# 串口通信库头文件
set(UART_FRAME_COMMAND_HEADERS
  include/exhand_read/uart_frame_command.hpp
)

# 创建串口通信库
add_library(uart_frame_command_lib ${UART_FRAME_COMMAND_SOURCES} ${UART_FRAME_COMMAND_HEADERS})
ament_target_dependencies(uart_frame_command_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# ROS2节点可执行文件
add_executable(exhand_node exhand_read/exhand_node.cpp)
ament_target_dependencies(exhand_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(exhand_node uart_frame_command_lib)

# 安装
install(TARGETS
  exhand_node
  uart_frame_command_lib
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

