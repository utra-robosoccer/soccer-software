// soccerbot real hardware interface (ros2_control SystemInterface).
//
// The REAL side of the sim/real boundary (blueprint §9). It does NOT close the
// torque loop — the Robostride actuators do that onboard. This interface streams
// a full MIT impedance command (q*, qd*, kp, kd, τ_ff) per joint to the Master
// STM32 over USB-CDC, and reads back per-joint state + a body IMU. If the Master
// heartbeat goes silent past the watchdog timeout it raises an error so the
// controller manager halts; the Master independently zeroes torque too.
//
// Contract: docs/architecture/jetson_master_protocol.md (DRAFT, pending firmware
// agreement). Wire structs live in wire_protocol.hpp.
#ifndef SOCCER_HARDWARE__SOCCERBOT_SERIAL_HARDWARE_HPP_
#define SOCCER_HARDWARE__SOCCERBOT_SERIAL_HARDWARE_HPP_

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

class SoccerbotSerialHardware : public hardware_interface::SystemInterface
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
  void send_command_frame();
  bool poll_state_frame();

  // ── Config (from <hardware><param> in the URDF) ──
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{1000000};
  std::chrono::milliseconds watchdog_timeout_{100};

  int fd_{-1};  // serial file descriptor (-1 = closed / no-MCU mode)
  std::chrono::steady_clock::time_point last_rx_{};
  uint16_t tx_seq_{0};

  // ── Per-joint command (full MIT) and state, sized from the URDF ──
  std::vector<double> cmd_q_;     // position target  (rad)
  std::vector<double> cmd_qd_;    // velocity target  (rad/s)
  std::vector<double> cmd_kp_;    // impedance stiffness
  std::vector<double> cmd_kd_;    // impedance damping
  std::vector<double> cmd_tau_;   // feed-forward torque (N·m)
  std::vector<double> state_q_;
  std::vector<double> state_qd_;
  std::vector<double> state_tau_;

  // ── Body IMU: 4 orientation + 3 gyro + 3 accel ──
  std::array<double, 10> imu_{};

  // Reassembly buffer for COBS-delimited frames spanning multiple reads.
  std::vector<uint8_t> rx_accum_;
};

}  // namespace soccer_hardware

#endif  // SOCCER_HARDWARE__SOCCERBOT_SERIAL_HARDWARE_HPP_
