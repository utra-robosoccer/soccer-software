cmake_minimum_required(VERSION 3.16)
project(soccer_strategy LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soccer_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(strategy_node
  src/strategy_node.cpp
  src/bt_nodes.cpp
  src/role_auction.cpp
)
target_include_directories(strategy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(strategy_node
  rclcpp behaviortree_cpp geometry_msgs std_msgs soccer_msgs ament_index_cpp
)

install(TARGETS strategy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_role_auction test/test_role_auction.cpp src/role_auction.cpp)
  target_include_directories(test_role_auction PUBLIC include)
  ament_target_dependencies(test_role_auction soccer_msgs)
endif()

ament_package()
