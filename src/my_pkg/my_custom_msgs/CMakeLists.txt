cmake_minimum_required(VERSION 3.20)
set(PROJECT_NAME my_custom_msgs)
project(${PROJECT_NAME} VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rosidl_default_generators
  std_msgs
  geographic_msgs
  nav_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

file(GLOB_RECURSE MSGS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB_RECURSE SRVS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
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

ament_package()
