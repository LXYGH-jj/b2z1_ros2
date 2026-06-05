cmake_minimum_required(VERSION 3.8)
project(b2z1_b2_lowcmd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(b2z1_b2_trajectories REQUIRED)

add_library(${PROJECT_NAME}_core
  src/common/motor_crc.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  unitree_go
  unitree_api
  b2z1_b2_trajectories
)

target_link_libraries(${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
)

add_executable(b2_real_lowcmd_stand_node
  src/b2_real_lowcmd_stand_node.cpp
)

target_include_directories(b2_real_lowcmd_stand_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(b2_real_lowcmd_stand_node
  rclcpp
  unitree_go
  unitree_api
  b2z1_b2_trajectories
)

target_link_libraries(b2_real_lowcmd_stand_node
  ${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
)

install(TARGETS
  ${PROJECT_NAME}_core
  b2_real_lowcmd_stand_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp unitree_go unitree_api nlohmann_json b2z1_b2_trajectories)

ament_package()
