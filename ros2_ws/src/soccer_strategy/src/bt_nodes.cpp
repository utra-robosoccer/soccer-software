#include "soccer_strategy/bt_nodes.hpp"

#include "soccer_msgs/msg/control_goal.hpp"

namespace soccer_strategy
{

void register_bt_nodes(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx)
{
  using soccer_msgs::msg::ControlGoal;
  using soccer_msgs::msg::GameState;

  // ── Conditions ──
  // The GameController is the ultimate authority: not PLAYING or penalized => halt.
  factory.registerSimpleCondition("IsNotHalted", [ctx](BT::TreeNode &) {
    const bool playing = ctx->gamestate == GameState::GAMESTATE_PLAYING;
    return (playing && !ctx->penalized) ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
  });

  factory.registerSimpleCondition("IsBallDetected", [ctx](BT::TreeNode &) {
    return ctx->ball_detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  // ── Actions (each emits a ControlGoal for the MPC) ──
  factory.registerSimpleAction("Idle", [ctx](BT::TreeNode &) {
    ctx->publish_goal(ControlGoal::MODE_IDLE, 0.0);
    return BT::NodeStatus::SUCCESS;
  });

  factory.registerSimpleAction("ScanForBall", [ctx](BT::TreeNode &) {
    ctx->publish_goal(ControlGoal::MODE_SCAN, 0.0);
    return BT::NodeStatus::SUCCESS;
  });

  factory.registerSimpleAction("TrackBall", [ctx](BT::TreeNode &) {
    ctx->publish_goal(ControlGoal::MODE_TRACK, ctx->ball_bearing);
    return BT::NodeStatus::SUCCESS;
  });
}

}  // namespace soccer_strategy
