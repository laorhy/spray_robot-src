cmake_minimum_required(VERSION 3.8)
project(pointcloud2mesh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io features surface)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMesh.srv"
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_to_mesh_server src/cloud_to_mesh_server.cpp)
ament_target_dependencies(cloud_to_mesh_server
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(cloud_to_mesh_server
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(cloud_to_mesh_server
  ${cpp_typesupport_target}
)

install(TARGETS cloud_to_mesh_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
