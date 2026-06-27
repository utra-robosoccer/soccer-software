// mpc_node — the model-based reference generator (L1, blueprint §4).
//
// On the full robot this is the footstep planner + ZMP/whole-body MPC that emits
// a physically grounded reference trajectory. On soccerbot (placeholder single joint) it reduces
// to a clamped-acceleration reference for `neck_pan` that realizes the high-level
// ControlGoal from Strategy:
//
//   MODE_IDLE  -> hold position
//   MODE_SCAN  -> sweep across the pan range to search / aid localization
//   MODE_TRACK -> drive the camera toward a target bearing (e.g. the ball)
//
// It publishes [q_ref, qd_ref] at 50 Hz; the ResidualRLController consumes it and
// adds the bounded RL residual. Keeping MPC and RL separate keeps the trajectory
// debuggable and rules-compliant while RL absorbs disturbances.
#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "soccer_msgs/msg/control_goal.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class MpcNode : public rclcpp::Node
{
public:
  MpcNode() : Node("mpc_node")
  {
    pan_limit_ = declare_parameter<double>("pan_limit", 1.57);
    max_vel_ = declare_parameter<double>("max_velocity", 4.0);
    max_acc_ = declare_parameter<double>("max_acceleration", 12.0);
    scan_rate_ = declare_parameter<double>("scan_rate", 0.6);  // Hz of the sweep

    ref_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "control/mpc_reference", 10);
    goal_sub_ = create_subscription<soccer_msgs::msg::ControlGoal>(
      "control/goal", 10,
      [this](const soccer_msgs::msg::ControlGoal::SharedPtr msg) { goal_ = *msg; });

    // 50 Hz reference band.
    timer_ = create_wall_timer(20ms, [this]() { step(); });
    RCLCPP_INFO(get_logger(), "mpc_node up: publishing reference at 50 Hz.");
  }

private:
  void step()
  {
    const double dt = 0.02;
    double q_target = q_ref_;

    switch (goal_.mode) {
      case soccer_msgs::msg::ControlGoal::MODE_IDLE:
        q_target = q_ref_;  // hold
        break;
      case soccer_msgs::msg::ControlGoal::MODE_SCAN:
        scan_phase_ += 2.0 * M_PI * scan_rate_ * dt;
        q_target = 0.9 * pan_limit_ * std::sin(scan_phase_);
        break;
      case soccer_msgs::msg::ControlGoal::MODE_TRACK:
        q_target = std::clamp(goal_.target_bearing, -pan_limit_, pan_limit_);
        break;
      default:
        q_target = q_ref_;
    }

    // Clamp velocity + acceleration to keep the reference physically realizable
    // (the role a real MPC plays — never command a step the actuator can't track).
    const double v_des = std::clamp((q_target - q_ref_) / dt, -max_vel_, max_vel_);
    const double a = std::clamp((v_des - qd_ref_) / dt, -max_acc_, max_acc_);
    qd_ref_ += a * dt;
    q_ref_ += qd_ref_ * dt;
    q_ref_ = std::clamp(q_ref_, -pan_limit_, pan_limit_);

    std_msgs::msg::Float64MultiArray out;
    out.data = {q_ref_, qd_ref_};
    ref_pub_->publish(out);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
  rclcpp::Subscription<soccer_msgs::msg::ControlGoal>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  soccer_msgs::msg::ControlGoal goal_;
  double q_ref_{0.0}, qd_ref_{0.0}, scan_phase_{0.0};
  double pan_limit_{1.57}, max_vel_{4.0}, max_acc_{12.0}, scan_rate_{0.6};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcNode>());
  rclcpp::shutdown();
  return 0;
}
