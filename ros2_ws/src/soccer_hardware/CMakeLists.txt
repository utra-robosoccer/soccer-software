cmake_minimum_required(VERSION 3.16)
project(soccer_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(DEPS hardware_interface pluginlib rclcpp rclcpp_lifecycle)

add_library(${PROJECT_NAME} SHARED
  src/soccerbot_sim_hardware.cpp
  src/soccerbot_serial_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})

# Export the plugins to pluginlib so the controller manager can load them.
pluginlib_export_plugin_description_file(hardware_interface soccer_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
