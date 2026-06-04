cmake_minimum_required(VERSION 3.8)
project(b2z1_b2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(b2z1_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME}_client
  src/ros2_b2_sport_client.cpp
)
target_include_directories(${PROJECT_NAME}_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_client
  rclcpp
  unitree_api
)
target_link_libraries(${PROJECT_NAME}_client
  nlohmann_json::nlohmann_json
)

add_executable(b2_bridge_node src/b2_bridge_node.cpp)
target_link_libraries(b2_bridge_node ${PROJECT_NAME}_client)
target_include_directories(b2_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(b2_bridge_node
  rclcpp
  unitree_api
  b2z1_msgs
)

install(TARGETS
  ${PROJECT_NAME}_client
  b2_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
