cmake_minimum_required(VERSION 3.16)
project(soccer_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soccer_msgs REQUIRED)

# ── 1. The residual-RL controller plugin (loaded by the controller manager) ──
add_library(residual_rl_controller SHARED
  src/residual_rl_controller.cpp
)
target_include_directories(residual_rl_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(residual_rl_controller
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools std_msgs soccer_msgs
)
pluginlib_export_plugin_description_file(controller_interface soccer_control_plugins.xml)

# ── 2. The MPC reference node (standalone executable) ──
add_executable(mpc_node src/mpc_node.cpp)
target_include_directories(mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mpc_node
  rclcpp std_msgs geometry_msgs soccer_msgs
)

install(TARGETS residual_rl_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mpc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(residual_rl_controller)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
