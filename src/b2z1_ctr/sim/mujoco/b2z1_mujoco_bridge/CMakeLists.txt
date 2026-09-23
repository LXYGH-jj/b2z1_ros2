cmake_minimum_required(VERSION 3.8)
project(b2z1_mujoco_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(b2z1_msgs REQUIRED)
find_package(b2z1_b2_lowcmd REQUIRED)
find_package(b2z1_b2_trajectories REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(b2z1_mujoco_bridge_node
  src/mujoco_bridge_node.cpp
)

target_include_directories(b2z1_mujoco_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(b2z1_mujoco_bridge_node
  rclcpp
  b2z1_msgs
  b2z1_b2_lowcmd
  b2z1_b2_trajectories
)

target_link_libraries(b2z1_mujoco_bridge_node
  unitree_sdk2
)

install(TARGETS
  b2z1_mujoco_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
