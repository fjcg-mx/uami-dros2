cmake_minimum_required(VERSION 3.20)
project(my_node_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_custom_msgs REQUIRED)

add_executable(status_node src/status_node.cpp)

# --- Include dirs (opcional pero recomendado) ---
target_include_directories(status_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Enlazar librerías ---
target_link_libraries(status_node
  rclcpp::rclcpp
  my_custom_msgs::my_custom_msgs__rosidl_typesupport_cpp
  my_custom_msgs::my_custom_msgs__rosidl_generator_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS status_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
