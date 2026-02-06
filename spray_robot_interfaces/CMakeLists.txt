cmake_minimum_required(VERSION 3.8)
project(spray_robot_interfaces)

# Default to C99 / C++14 if needed by generators
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Dependencies referenced in msg/srv/action
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# -----------------------------
# List your interface files here
# -----------------------------
set(msg_files
  "msg/SprayPose.msg"
  "msg/SprayPath.msg"
  "msg/BlockStatus.msg"
)

# If you have srv, add them here (or keep empty)
set(srv_files
  # "srv/YourService.srv"
)

# If you have actions, add them here (or keep empty)
set(action_files
  "action/SprayBlock.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------
# Tests / Lint (recommended)
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
