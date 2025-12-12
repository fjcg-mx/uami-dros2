cmake_minimum_required(VERSION 3.20)
project(test_rclcpp_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# Crear el ejecutable
add_executable(test_rclcpp_tf2 src/test_rclcpp_tf2.cpp)

# Vincula correctamente las bibliotecas
target_link_libraries(test_rclcpp_tf2
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

install(TARGETS
  test_rclcpp_tf2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
