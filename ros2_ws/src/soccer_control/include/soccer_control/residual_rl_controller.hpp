// ResidualRLController — the L1 policy layer as a ros2_control controller.
//
// Runs inside the controller manager at 100 Hz. Each update:
//   1. read measured joint state (q, qd, τ) from state interfaces
//   2. read the latest MPC reference (q*, qd*) from a topic
//   3. evaluate the bounded RL residual Δq = clamp(policy(obs), ±limit)
//   4. write q_target = q* + Δq to the position command interface (+ ff effort)
// The actual 1 kHz PD tracking of q_target happens on the MCU (blueprint §4).
#ifndef SOCCER_CONTROL__RESIDUAL_RL_CONTROLLER_HPP_
#define SOCCER_CONTROL__RESIDUAL_RL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "soccer_control/policy_runner.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace soccer_control
{

class ResidualRLController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string joint_;
  std::string reference_topic_;
  double residual_limit_{0.20};
  double effort_limit_{3.0};

  PolicyRunner policy_;

  // [q_ref, qd_ref] from the MPC node, double-buffered for RT safety.
  realtime_tools::RealtimeBuffer<std::array<double, 2>> mpc_ref_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
};

}  // namespace soccer_control

#endif  // SOCCER_CONTROL__RESIDUAL_RL_CONTROLLER_HPP_
