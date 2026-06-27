// Behavior Tree leaf nodes for soccerbot strategy (blueprint §6).
//
// The tree reads a shared StrategyContext (the world-model snapshot updated by
// the strategy node's ROS subscriptions) and turns intent into a ControlGoal for
// the MPC. We use BehaviorTree.CPP's "simple" condition/action registration with
// a captured context, which keeps the leaves tiny and Groot2-debuggable.
#ifndef SOCCER_STRATEGY__BT_NODES_HPP_
#define SOCCER_STRATEGY__BT_NODES_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "soccer_msgs/msg/game_state.hpp"

namespace soccer_strategy
{

// Shared snapshot the BT reads each tick. Filled by strategy_node subscriptions.
struct StrategyContext
{
  uint8_t gamestate{soccer_msgs::msg::GameState::GAMESTATE_INITIAL};
  bool penalized{false};
  bool ball_detected{false};
  double ball_bearing{0.0};    // rad, base frame
  double ball_distance{1e9};   // m
  uint8_t role{0};

  // Sink the actions call to drive control (wired to a ControlGoal publisher).
  std::function<void(uint8_t mode, double bearing)> publish_goal;
};
using ContextPtr = std::shared_ptr<StrategyContext>;

// Registers all condition/action leaves, binding them to the shared context.
void register_bt_nodes(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx);

}  // namespace soccer_strategy

#endif  // SOCCER_STRATEGY__BT_NODES_HPP_
