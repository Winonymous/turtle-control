cmake_minimum_required(VERSION 3.8)
project(turtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtle_control REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MessageCounter.msg"
  "msg/Turtle2Available.msg"
  "srv/Stop.srv"
  "srv/AngleBetweenPoint.srv"
  "action/Move.action"
  "action/MovePosition.action"
)

add_executable(avoid_wall src/avoid_wall.cpp)
ament_target_dependencies(avoid_wall rclcpp geometry_msgs turtlesim)

add_executable(counter src/counter.cpp)
ament_target_dependencies(counter rclcpp turtle_control)

add_executable(move_linear src/move_linear.cpp)
ament_target_dependencies(move_linear rclcpp geometry_msgs turtlesim)

add_executable(turn_angle src/turn_angle.cpp)
ament_target_dependencies(turn_angle rclcpp geometry_msgs turtlesim)

add_executable(euclidean_distance_client src/euclidean_distance_client.cpp)
ament_target_dependencies(euclidean_distance_client rclcpp example_interfaces)

add_executable(euclidean_distance_server src/euclidean_distance_server.cpp)
ament_target_dependencies(euclidean_distance_server rclcpp turtlesim example_interfaces)

add_executable(stop_robot_server src/stop_robot_server.cpp)
ament_target_dependencies(stop_robot_server rclcpp turtle_control)

add_executable(move_action_server src/move_action_server.cpp)
ament_target_dependencies(move_action_server turtle_control rclcpp rclcpp_action turtlesim geometry_msgs)

install(TARGETS
  move_action_server
  avoid_wall
  move_linear
  turn_angle
  counter
  euclidean_distance_client
  euclidean_distance_server
  stop_robot_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_point_server.py
  scripts/move_point_action.py
  scripts/avoid_wall.py
  scripts/move_linear.py
  scripts/turn_angle.py
  scripts/counter.py
  scripts/drawtriangle.py
  scripts/euclidean_distance_client.py
  scripts/euclidean_distance_server.py
  scripts/stop_robot_client.py
  scripts/angle_between_point.py
  scripts/move_point_server.py
  scripts/turtle_catcher.py
  scripts/turtle_catcher_gamemaster.py
  scripts/turtle_catcher_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
