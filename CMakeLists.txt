cmake_minimum_required(VERSION 3.8)
project(ros2_tf_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ament_index_cpp REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FeatureDetection.srv"
  "srv/SimpleTrajectory.srv"
  "action/SimpleNavigation.action"
  DEPENDENCIES geometry_msgs
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

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(feature_detection_service_server src/feature_detection_service_server.cpp)
ament_target_dependencies(feature_detection_service_server
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  rosidl_default_runtime
  yaml-cpp
  rcpputils
)

add_executable(simple_trajectory_service_server src/simple_trajectory_service_server.cpp)
ament_target_dependencies(simple_trajectory_service_server
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  rosidl_default_runtime
  yaml-cpp
  ament_index_cpp
)

add_executable(simple_navigation_action_server src/simple_navigation_action_server.cpp)
ament_target_dependencies(simple_navigation_action_server
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  gazebo_msgs
  rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(feature_detection_service_server ${cpp_typesupport_target} yaml-cpp)
target_link_libraries(simple_trajectory_service_server ${cpp_typesupport_target} yaml-cpp)
target_link_libraries(simple_navigation_action_server ${cpp_typesupport_target})

install(TARGETS
  feature_detection_service_server
  simple_trajectory_service_server
  simple_navigation_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  action
  frames
  launch
  rviz
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
