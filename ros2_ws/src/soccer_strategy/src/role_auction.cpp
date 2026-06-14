#include "soccer_strategy/role_auction.hpp"

#include "soccer_msgs/msg/role_bid.hpp"

namespace soccer_strategy
{

uint8_t assign_role(uint8_t my_id, const std::vector<Bid> & bids, uint8_t goalie_id)
{
  using soccer_msgs::msg::RoleBid;

  if (my_id == goalie_id) {
    return RoleBid::ROLE_GOALIE;
  }

  // Find the active, non-goalie robot with the lowest ball cost. Ties are broken
  // by player_id so the result is identical on every robot (no master needed).
  bool found = false;
  uint8_t striker_id = 0;
  double best_cost = 0.0;
  for (const auto & b : bids) {
    if (!b.active || b.player_id == goalie_id) {
      continue;
    }
    if (!found || b.ball_cost < best_cost ||
        (b.ball_cost == best_cost && b.player_id < striker_id))
    {
      found = true;
      best_cost = b.ball_cost;
      striker_id = b.player_id;
    }
  }

  if (found && my_id == striker_id) {
    return RoleBid::ROLE_STRIKER;
  }
  return RoleBid::ROLE_SUPPORTER;
}

}  // namespace soccer_strategy
