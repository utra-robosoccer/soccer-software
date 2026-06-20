cmake_minimum_required(VERSION 3.8)
project(soccer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── Perception ──
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/FieldFeature.msg"
  "msg/FieldFeatureArray.msg"
  # ── Mission / referee ──
  "msg/GameState.msg"
  # ── Strategy / control hand-off ──
  "msg/ControlGoal.msg"
  # ── Team communication / role auction ──
  "msg/RoleBid.msg"
  "msg/TeamData.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
