cmake_minimum_required(VERSION 3.8)
project(macortex_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseStampedArray.msg"
  "msg/QuadPose.msg"
  "msg/QuadPoseList.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Cortex SDK static library (built in the colcon build tree, not in-source).
add_library(cortex_sdk STATIC
  lib/src/m3x3.cpp
  lib/src/cortex_socket.cpp
  lib/src/cortex_unpack.cpp
  lib/src/cortex.cpp
)
target_include_directories(cortex_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/src>
)
target_link_libraries(cortex_sdk pthread m)

add_executable(macortex_bridge_node src/macortex_bridge.cpp)
set_target_properties(macortex_bridge_node PROPERTIES OUTPUT_NAME macortex_bridge)
target_include_directories(macortex_bridge_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(macortex_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
)
target_link_libraries(macortex_bridge_node cortex_sdk)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(macortex_bridge_node "${cpp_typesupport_target}")

install(TARGETS macortex_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
