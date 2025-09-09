cmake_minimum_required(VERSION 3.8)
project(hd_disassembly)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)  
find_package(rclcpp REQUIRED)                           # For C++ nodes :contentReference[oaicite:0]{index=0}  
find_package(rosidl_default_generators REQUIRED)        # For generating our custom interfaces :contentReference[oaicite:1]{index=1}  
find_package(geometry_msgs REQUIRED)                    # For PoseStamped, PointStamped :contentReference[oaicite:2]{index=2}  

# 1) List your action and service files:
set(action_files
  "action/MoveToPose.action"
  "action/Probing.action"
  "action/Unscrewing.action"
)

set(srv_files
  "srv/TransformPoint.srv"
  "srv/GetSequence.srv"
  "srv/GetPlan.srv"
)

# List your .msg files:
set(msg_files
  "msg/BoundingBox.msg"
  "msg/DetectedPart.msg"
  "msg/TrackedPart.msg"
)

# 2) Generate interfaces:
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)  # NOTE: duplicate dependencies here for services/actions :contentReference[oaicite:3]{index=3}

# 3) If you have C++ executables, link against the generated code:
install(PROGRAMS
  src/ready_pose.py
  src/vision_detector_node.py
  src/vision_detector_node_detr.py
  src/vision_detector_yolo_sahi.py
  src/tool_cam_yolo_node.py
  src/transform_node.py
  src/planner_node.py
  src/marker_publisher_node.py
  src/robot_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4) Install generated interface files so downstream packages can find them:
install(
  DIRECTORY action srv launch config
  DESTINATION share/${PROJECT_NAME}
)



ament_export_dependencies(rosidl_default_runtime)  # Export interface runtime :contentReference[oaicite:4]{index=4}  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
