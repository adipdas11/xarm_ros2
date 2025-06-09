cmake_minimum_required(VERSION 3.8)
project(disassembly_v3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ament and ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# message & action dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# generate .srv and .action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Services
  "srv/GetPartGraph.srv"
  "srv/GetSequence.srv"
  "srv/TransformPose.srv"

  # Actions
  "action/ApproachUntilContact.action"
  "action/MoveToPose.action"
  "action/UnscrewScrew.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    action_msgs
    builtin_interfaces
)

# install Python scripts as executables
install(PROGRAMS
  scripts/vision_graph_node.py
  scripts/sequence_planner_node.py
  scripts/pose_transform_service.py
  scripts/velocity_servo_action_server.py
  scripts/contact_approach_action_server.py
  scripts/unscrew_action_server.py
  scripts/disassembly_orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
