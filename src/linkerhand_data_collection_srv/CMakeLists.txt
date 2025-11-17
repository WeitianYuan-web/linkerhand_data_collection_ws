cmake_minimum_required(VERSION 3.8)
project(linkerhand_data_collection_srv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Internal.srv"
  DEPENDENCIES std_msgs
)

# No Python package to install - scripts are installed directly

# Install Python executables
install(PROGRAMS
  scripts/linkerhand_data_collection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install utils directory
install(DIRECTORY
  scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install service files
install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
