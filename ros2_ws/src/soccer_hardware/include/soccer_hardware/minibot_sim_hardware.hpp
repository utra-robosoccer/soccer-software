// MiniBot simulation hardware interface (ros2_control SystemInterface).
//
// Integrates the single neck_pan joint with a simple second-order model and
// synthesizes IMU readings. This is the SIM side of the sim/real boundary
// (blueprint §10): the controllers above it cannot tell whether they are
// driving this class or the real serial hardware.
#ifndef SOCCER_HARDWARE__MINIBOT_SIM_HARDWARE_HPP_
#define SOCCER_HARDWARE__MINIBOT_SIM_HARDWARE_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace soccer_hardware
{

class MinibotSimHardware : public hardware_interface::SystemInterface
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
  // ── The ONE motor: state + command ──
  double pos_{0.0};
  double vel_{0.0};
  double eff_{0.0};
  double cmd_pos_{0.0};
  double cmd_eff_{0.0};

  // ── The ONE IMU: 4 orientation + 3 gyro + 3 accel ──
  std::array<double, 10> imu_{};

  // Simple model parameters (critically-damped position tracking).
  double kp_{120.0};
  double kd_{12.0};
  double inertia_{0.01};
  double damping_{0.05};
  double pos_min_{-1.57};
  double pos_max_{1.57};
};

}  // namespace soccer_hardware

#endif  // SOCCER_HARDWARE__MINIBOT_SIM_HARDWARE_HPP_
