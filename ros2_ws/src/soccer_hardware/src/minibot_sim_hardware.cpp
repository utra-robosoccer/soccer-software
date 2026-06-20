#include "soccer_hardware/minibot_sim_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace soccer_hardware
{

hardware_interface::CallbackReturn MinibotSimHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // MiniBot has exactly one joint. Read its limits from the URDF if present.
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MinibotSimHardware"),
      "Expected exactly 1 joint (neck_pan), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & cmd_ifaces = info_.joints[0].command_interfaces;
  for (const auto & ci : cmd_ifaces) {
    if (ci.name == hardware_interface::HW_IF_POSITION) {
      if (!ci.min.empty()) pos_min_ = std::stod(ci.min);
      if (!ci.max.empty()) pos_max_ = std::stod(ci.max);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MinibotSimHardware"),
    "Initialized SIM hardware for joint '%s' (limits [%.2f, %.2f] rad)",
    info_.joints[0].name.c_str(), pos_min_, pos_max_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MinibotSimHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const std::string joint = info_.joints[0].name;
  state_interfaces.emplace_back(joint, hardware_interface::HW_IF_POSITION, &pos_);
  state_interfaces.emplace_back(joint, hardware_interface::HW_IF_VELOCITY, &vel_);
  state_interfaces.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &eff_);

  // IMU sensor state interfaces (must match the <sensor> block in the URDF).
  if (!info_.sensors.empty()) {
    const std::string s = info_.sensors[0].name;
    const std::array<std::string, 10> names = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
    for (size_t i = 0; i < names.size(); ++i) {
      state_interfaces.emplace_back(s, names[i], &imu_[i]);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MinibotSimHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  const std::string joint = info_.joints[0].name;
  command_interfaces.emplace_back(joint, hardware_interface::HW_IF_POSITION, &cmd_pos_);
  command_interfaces.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &cmd_eff_);
  return command_interfaces;
}

hardware_interface::CallbackReturn MinibotSimHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pos_ = 0.0;
  vel_ = 0.0;
  eff_ = 0.0;
  cmd_pos_ = 0.0;
  cmd_eff_ = 0.0;
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};  // identity + gravity
  RCLCPP_INFO(rclcpp::get_logger("MinibotSimHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MinibotSimHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MinibotSimHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Integrate the joint with a PD-to-target + feed-forward effort model. This
  // mimics what the MCU's 1 kHz loop does on the real robot, so the controller
  // above sees consistent dynamics on both sides of the boundary.
  const double dt = std::max(period.seconds(), 1e-4);
  const double torque =
    kp_ * (cmd_pos_ - pos_) - kd_ * vel_ + cmd_eff_ - damping_ * vel_;
  const double accel = torque / inertia_;
  vel_ += accel * dt;
  pos_ += vel_ * dt;

  // Enforce joint limits (hard stop).
  if (pos_ < pos_min_) { pos_ = pos_min_; vel_ = std::max(0.0, vel_); }
  if (pos_ > pos_max_) { pos_ = pos_max_; vel_ = std::min(0.0, vel_); }

  eff_ = torque;  // "measured" effort (what current-sense would report)

  // Synthesize a static-base IMU: identity orientation, ~0 gyro, gravity on z.
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, vel_ * 0.0, 0.0, 0.0, 9.81};
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MinibotSimHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Commands are consumed directly by the model in read(); nothing to send.
  return hardware_interface::return_type::OK;
}

}  // namespace soccer_hardware

PLUGINLIB_EXPORT_CLASS(
  soccer_hardware::MinibotSimHardware, hardware_interface::SystemInterface)
