// strategy_node — L4 per-robot Behavior Tree + decentralized role auction
// (blueprint §6). Ticks a role-specific BT at 10 Hz, turning the world model into
// a ControlGoal for the MPC. The role is chosen each tick by the decentralized
// auction over all teammates' bids — no master.
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "soccer_msgs/msg/control_goal.hpp"
#include "soccer_msgs/msg/game_state.hpp"
#include "soccer_msgs/msg/role_bid.hpp"
#include "soccer_msgs/msg/team_data.hpp"
#include "soccer_strategy/bt_nodes.hpp"
#include "soccer_strategy/role_auction.hpp"

using namespace std::chrono_literals;

class StrategyNode : public rclcpp::Node
{
public:
  StrategyNode() : Node("strategy_node")
  {
    player_id_ = declare_parameter<int>("player_id", 1);
    goalie_id_ = declare_parameter<int>("goalie_id", 1);
    ball_timeout_ = declare_parameter<double>("ball_timeout", 1.0);

    ctx_ = std::make_shared<soccer_strategy::StrategyContext>();
    goal_pub_ = create_publisher<soccer_msgs::msg::ControlGoal>("control/goal", 10);
    bid_pub_ = create_publisher<soccer_msgs::msg::RoleBid>("strategy/role_bid", 10);
    ctx_->publish_goal = [this](uint8_t mode, double bearing) {
      soccer_msgs::msg::ControlGoal g;
      g.header.stamp = now();
      g.mode = mode;
      g.target_bearing = bearing;
      goal_pub_->publish(g);
    };

    gc_sub_ = create_subscription<soccer_msgs::msg::GameState>(
      "gc/game_state", 10,
      [this](const soccer_msgs::msg::GameState::SharedPtr m) {
        ctx_->gamestate = m->gamestate;
        ctx_->penalized = m->penalized;
      });
    ball_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "ball/point", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr m) {
        ctx_->ball_bearing = std::atan2(m->point.y, m->point.x);
        ctx_->ball_distance = std::hypot(m->point.x, m->point.y);
        last_ball_ = now();
      });
    // Team data is shared on a GLOBAL (un-namespaced) topic so every robot sees
    // every bid and computes the SAME assignment (blueprint §6, §8).
    team_sub_ = create_subscription<soccer_msgs::msg::TeamData>(
      "/team_data", 20,
      [this](const soccer_msgs::msg::TeamData::SharedPtr m) {
        bids_[m->player_id] = {
          static_cast<uint8_t>(m->player_id), m->bid.cost,
          m->status != soccer_msgs::msg::TeamData::STATUS_INACTIVE};
      });

    build_factory();
    load_tree(soccer_msgs::msg::RoleBid::ROLE_SUPPORTER);  // default until first auction
    timer_ = create_wall_timer(100ms, [this]() { step(); });  // 10 Hz
    RCLCPP_INFO(get_logger(), "strategy_node up (player %d).", player_id_);
  }

private:
  void build_factory()
  {
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    soccer_strategy::register_bt_nodes(*factory_, ctx_);
    tree_dir_ = ament_index_cpp::get_package_share_directory("soccer_strategy") + "/trees/";
  }

  void load_tree(uint8_t role)
  {
    using soccer_msgs::msg::RoleBid;
    std::string file = "supporter.xml";
    if (role == RoleBid::ROLE_STRIKER) file = "striker.xml";
    else if (role == RoleBid::ROLE_GOALIE) file = "goalie.xml";
    tree_ = std::make_unique<BT::Tree>(factory_->createTreeFromFile(tree_dir_ + file));
    current_role_ = role;
    ctx_->role = role;
    RCLCPP_INFO(get_logger(), "Loaded role tree: %s", file.c_str());
  }

  void step()
  {
    // 1. Ball freshness.
    const bool fresh = (now() - last_ball_).seconds() < ball_timeout_;
    ctx_->ball_detected = fresh;
    const double my_cost = fresh ? ctx_->ball_distance : 1e6;

    // 2. Broadcast our own bid (teamcomm packages it onto /team_data).
    soccer_msgs::msg::RoleBid bid;
    bid.cost = my_cost;
    bid.role = current_role_;
    bid_pub_->publish(bid);
    bids_[player_id_] = {static_cast<uint8_t>(player_id_), my_cost, !ctx_->penalized};

    // 3. Run the decentralized auction over all known bids.
    std::vector<soccer_strategy::Bid> all;
    all.reserve(bids_.size());
    for (auto & [id, b] : bids_) all.push_back(b);
    const uint8_t role = soccer_strategy::assign_role(
      static_cast<uint8_t>(player_id_), all, static_cast<uint8_t>(goalie_id_));
    if (role != current_role_) {
      load_tree(role);  // hot-swap behavior on role change
    }

    // 4. Tick the tree.
    tree_->tickOnce();
  }

  int player_id_{1}, goalie_id_{1};
  double ball_timeout_{1.0};
  uint8_t current_role_{255};

  soccer_strategy::ContextPtr ctx_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::string tree_dir_;
  std::map<int, soccer_strategy::Bid> bids_;
  rclcpp::Time last_ball_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<soccer_msgs::msg::ControlGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<soccer_msgs::msg::RoleBid>::SharedPtr bid_pub_;
  rclcpp::Subscription<soccer_msgs::msg::GameState>::SharedPtr gc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_sub_;
  rclcpp::Subscription<soccer_msgs::msg::TeamData>::SharedPtr team_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StrategyNode>());
  rclcpp::shutdown();
  return 0;
}
