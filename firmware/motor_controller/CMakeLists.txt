cmake_minimum_required(VERSION 3.16)
project(minibot_motor_controller C)

set(CMAKE_C_STANDARD 11)

option(BUILD_HOST_TESTS "Build host-side unit tests of the control math" OFF)
option(BUILD_MCU_FIRMWARE "Cross-compile the MCU firmware image" OFF)

# Portable, hardware-free control math — shared by the MCU and the host tests.
add_library(minibot_control STATIC
  src/pd_controller.c
  src/watchdog.c
)
target_include_directories(minibot_control PUBLIC include)

# ── Host unit tests (run in CI: cmake -DBUILD_HOST_TESTS=ON && ctest) ──
if(BUILD_HOST_TESTS)
  enable_testing()
  add_executable(test_control test/test_pd_controller.c)
  target_link_libraries(test_control PRIVATE minibot_control)
  add_test(NAME control_math COMMAND test_control)
endif()

# ── MCU firmware image (needs an arm-none-eabi toolchain + BSP) ──
if(BUILD_MCU_FIRMWARE)
  add_executable(minibot_firmware src/main.c)
  target_link_libraries(minibot_firmware PRIVATE minibot_control)
  # The board-support package (HAL, micro-ROS, motor driver) is linked here by
  # the firmware team's toolchain file.
endif()
