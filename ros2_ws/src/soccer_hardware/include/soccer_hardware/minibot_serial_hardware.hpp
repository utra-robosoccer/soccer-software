// MiniBot real hardware interface (ros2_control SystemInterface).
//
// The REAL side of the sim/real boundary (blueprint §9). It does NOT close the
// 1 kHz loop itself — that lives on the MCU. Instead it streams position/effort
// commands to the STM32/Teensy over serial and reads back joint state + IMU.
// If the MCU heartbeat goes silent past the watchdog timeout it raises an error
// so the controller manager halts; the MCU independently zeroes torque too.
#ifndef SOCCER_HARDWARE__MINIBOT_SERIAL_HARDWARE_HPP_
#define SOCCER_HARDWARE__MINIBOT_SERIAL_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace soccer_hardware
{

// CRC16-CCITT (poly 0x1021, init 0xFFFF) — identical to soccer-firmware's wire
// format so this host interface and the MCU agree on framing.
uint16_t crc16_ccitt(const uint8_t * data, size_t len);

class MinibotSerialHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_port();
  void close_port();

  // ── Config (from <hardware><param> in the URDF) ──
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{1000000};
  std::chrono::milliseconds watchdog_timeout_{100};

  int fd_{-1};  // serial file descriptor (-1 = closed)
  std::chrono::steady_clock::time_point last_rx_{};

  // ── The ONE motor: state + command ──
  double pos_{0.0}, vel_{0.0}, eff_{0.0};
  double cmd_pos_{0.0}, cmd_eff_{0.0};

  // ── The ONE IMU ──
  std::array<double, 10> imu_{};
};

}  // namespace soccer_hardware

#endif  // SOCCER_HARDWARE__MINIBOT_SERIAL_HARDWARE_HPP_
