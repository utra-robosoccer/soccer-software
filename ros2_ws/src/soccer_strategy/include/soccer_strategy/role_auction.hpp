// Decentralized role auction (blueprint §6).
//
// There is NO master. Every robot broadcasts its bid (cost = distance to ball)
// over team comms; every robot then runs THIS same deterministic assignment over
// the full set of bids and arrives at the identical result. If a teammate drops
// out its bid disappears and the next tick re-assigns automatically.
#ifndef SOCCER_STRATEGY__ROLE_AUCTION_HPP_
#define SOCCER_STRATEGY__ROLE_AUCTION_HPP_

#include <cstdint>
#include <vector>

namespace soccer_strategy
{

struct Bid
{
  uint8_t player_id;
  double ball_cost;  // distance-to-ball (lower = stronger claim to striker)
  bool active;       // false when penalized / disconnected
};

// Returns the role (soccer_msgs::RoleBid::ROLE_*) assigned to `my_id` given the
// full set of `bids`. `goalie_id` is statically designated (rules-friendly).
uint8_t assign_role(uint8_t my_id, const std::vector<Bid> & bids, uint8_t goalie_id);

}  // namespace soccer_strategy

#endif  // SOCCER_STRATEGY__ROLE_AUCTION_HPP_
