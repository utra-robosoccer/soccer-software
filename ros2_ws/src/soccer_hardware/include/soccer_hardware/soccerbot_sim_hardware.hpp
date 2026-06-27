// soccerbot simulation hardware interface (ros2_control SystemInterface).
//
// The SIM side of the sim/real boundary (blueprint §10). It integrates each
// joint with the SAME MIT impedance law the Robostride actuators apply onboard
// (τ = kp·(q*−q) + kd·(qd*−qd) + τ_ff), so controllers above the boundary see
// consistent dynamics whether they drive this class or the real serial hardware.
// Generic over N joints — everything is sized from the URDF.
#ifndef SOCCER_HARDWARE__SOCCERBOT_SIM_HARDWARE_HPP_
#define SOCCER_HARDWARE__SOCCERBOT_SIM_HARDWARE_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace soccer_hardware
{

class SoccerbotSimHardware : public hardware_interface::SystemInterface
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
  // ── Per-joint command (full MIT) and state, sized from the URDF ──
  std::vector<double> cmd_q_;
  std::vector<double> cmd_qd_;
  std::vector<double> cmd_kp_;
  std::vector<double> cmd_kd_;
  std::vector<double> cmd_tau_;
  std::vector<double> state_q_;
  std::vector<double> state_qd_;
  std::vector<double> state_tau_;

  // Per-joint soft limits and a simple plant (inertia + viscous damping).
  std::vector<double> pos_min_;
  std::vector<double> pos_max_;
  std::vector<double> inertia_;
  std::vector<double> damping_;

  // ── Body IMU: 4 orientation + 3 gyro + 3 accel ──
  std::array<double, 10> imu_{};
};

}  // namespace soccer_hardware

#endif  // SOCCER_HARDWARE__SOCCERBOT_SIM_HARDWARE_HPP_
