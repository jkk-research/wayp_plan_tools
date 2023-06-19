cmake_minimum_required(VERSION 3.8)
project(wayp_plan_tools)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  
)

include_directories(
  include
)

add_executable(waypoint_saver src/waypoint_saver.cpp)
ament_target_dependencies(waypoint_saver  ${ament_dependencies} )
add_executable(waypoint_loader src/waypoint_loader.cpp)
ament_target_dependencies(waypoint_loader  ${ament_dependencies} )
add_executable(waypoint_to_target src/waypoint_to_target.cpp)
ament_target_dependencies(waypoint_to_target  ${ament_dependencies} )
add_executable(single_goal_pursuit src/single_goal_pursuit.cpp)
ament_target_dependencies(single_goal_pursuit  ${ament_dependencies} )
add_executable(multiple_goal_pursuit src/multiple_goal_pursuit.cpp)
ament_target_dependencies(multiple_goal_pursuit  ${ament_dependencies} )
add_executable(stanley_control src/stanley_control.cpp)
ament_target_dependencies(stanley_control  ${ament_dependencies} )
add_executable(echo_metrics src/echo_metrics.cpp)
ament_target_dependencies(echo_metrics  ${ament_dependencies} )


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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  waypoint_saver
  waypoint_loader
  waypoint_to_target
  single_goal_pursuit
  multiple_goal_pursuit
  stanley_control
  echo_metrics
  DESTINATION lib/${PROJECT_NAME})
ament_package()
