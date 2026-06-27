#include "soccer_hardware/soccerbot_sim_hardware.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace soccer_hardware
{

namespace
{
constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

double joint_param_or(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  double fallback)
{
  auto it = joint.parameters.find(key);
  return it != joint.parameters.end() ? std::stod(it->second) : fallback;
}
}  // namespace

hardware_interface::CallbackReturn SoccerbotSimHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  cmd_q_.assign(n, 0.0);
  cmd_qd_.assign(n, 0.0);
  cmd_kp_.assign(n, 0.0);
  cmd_kd_.assign(n, 0.0);
  cmd_tau_.assign(n, 0.0);
  state_q_.assign(n, 0.0);
  state_qd_.assign(n, 0.0);
  state_tau_.assign(n, 0.0);
  pos_min_.assign(n, -3.14159);
  pos_max_.assign(n, 3.14159);
  inertia_.assign(n, 0.01);
  damping_.assign(n, 0.05);

  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];
    // Position limits come from the position command interface min/max if set.
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        if (!ci.min.empty()) pos_min_[i] = std::stod(ci.min);
        if (!ci.max.empty()) pos_max_[i] = std::stod(ci.max);
      }
    }
    // Optional per-joint plant params (sim-only) from the URDF.
    inertia_[i] = joint_param_or(joint, "sim_inertia", inertia_[i]);
    damping_[i] = joint_param_or(joint, "sim_damping", damping_[i]);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SoccerbotSimHardware"),
    "Initialised SIM hardware for %zu joints.", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SoccerbotSimHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & j = info_.joints[i].name;
    si.emplace_back(j, hardware_interface::HW_IF_POSITION, &state_q_[i]);
    si.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &state_qd_[i]);
    si.emplace_back(j, hardware_interface::HW_IF_EFFORT, &state_tau_[i]);
  }
  if (!info_.sensors.empty()) {
    const std::string & s = info_.sensors[0].name;
    static const std::array<std::string, 10> names = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
    for (size_t i = 0; i < names.size(); ++i) {
      si.emplace_back(s, names[i], &imu_[i]);
    }
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
SoccerbotSimHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & j = info_.joints[i].name;
    ci.emplace_back(j, hardware_interface::HW_IF_POSITION, &cmd_q_[i]);
    ci.emplace_back(j, hardware_interface::HW_IF_VELOCITY, &cmd_qd_[i]);
    ci.emplace_back(j, HW_IF_KP, &cmd_kp_[i]);
    ci.emplace_back(j, HW_IF_KD, &cmd_kd_[i]);
    ci.emplace_back(j, hardware_interface::HW_IF_EFFORT, &cmd_tau_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn SoccerbotSimHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(state_q_.begin(), state_q_.end(), 0.0);
  std::fill(state_qd_.begin(), state_qd_.end(), 0.0);
  std::fill(state_tau_.begin(), state_tau_.end(), 0.0);
  std::fill(cmd_q_.begin(), cmd_q_.end(), 0.0);
  std::fill(cmd_qd_.begin(), cmd_qd_.end(), 0.0);
  std::fill(cmd_tau_.begin(), cmd_tau_.end(), 0.0);
  // Default to a mild critically-damped impedance so the joints hold pose even
  // before a controller commands gains. The real motors freewheel (kp=kd=0).
  std::fill(cmd_kp_.begin(), cmd_kp_.end(), 120.0);
  std::fill(cmd_kd_.begin(), cmd_kd_.end(), 12.0);
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
  RCLCPP_INFO(rclcpp::get_logger("SoccerbotSimHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoccerbotSimHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SoccerbotSimHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = std::max(period.seconds(), 1e-4);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // MIT impedance law — the same one the Robostride applies onboard.
    const double torque =
      cmd_kp_[i] * (cmd_q_[i] - state_q_[i]) +
      cmd_kd_[i] * (cmd_qd_[i] - state_qd_[i]) +
      cmd_tau_[i] - damping_[i] * state_qd_[i];
    const double accel = torque / inertia_[i];
    state_qd_[i] += accel * dt;
    state_q_[i] += state_qd_[i] * dt;

    if (state_q_[i] < pos_min_[i]) { state_q_[i] = pos_min_[i]; state_qd_[i] = std::max(0.0, state_qd_[i]); }
    if (state_q_[i] > pos_max_[i]) { state_q_[i] = pos_max_[i]; state_qd_[i] = std::min(0.0, state_qd_[i]); }

    state_tau_[i] = torque;  // "measured" effort
  }
  // Static-base IMU: identity orientation, ~0 gyro, gravity on z.
  imu_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoccerbotSimHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Commands are consumed by the model in read(); nothing to transmit.
  return hardware_interface::return_type::OK;
}

}  // namespace soccer_hardware

PLUGINLIB_EXPORT_CLASS(
  soccer_hardware::SoccerbotSimHardware, hardware_interface::SystemInterface)
