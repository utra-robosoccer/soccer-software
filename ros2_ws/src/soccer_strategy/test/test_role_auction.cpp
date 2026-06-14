// Unit tests for the decentralized role auction.
#include <gtest/gtest.h>

#include <vector>

#include "soccer_msgs/msg/role_bid.hpp"
#include "soccer_strategy/role_auction.hpp"

using soccer_msgs::msg::RoleBid;
using soccer_strategy::assign_role;
using soccer_strategy::Bid;

TEST(RoleAuction, GoalieIsStaticallyAssigned)
{
  std::vector<Bid> bids = {{1, 0.5, true}, {2, 3.0, true}};
  EXPECT_EQ(assign_role(1, bids, /*goalie_id=*/1), RoleBid::ROLE_GOALIE);
}

TEST(RoleAuction, ClosestOutfieldRobotBecomesStriker)
{
  // Robot 1 is goalie; among 2 and 3, robot 3 is closer to the ball.
  std::vector<Bid> bids = {{1, 9.0, true}, {2, 2.5, true}, {3, 1.0, true}};
  EXPECT_EQ(assign_role(3, bids, 1), RoleBid::ROLE_STRIKER);
  EXPECT_EQ(assign_role(2, bids, 1), RoleBid::ROLE_SUPPORTER);
}

TEST(RoleAuction, DropoutTriggersReassignment)
{
  // Striker (robot 3) drops out -> robot 2 must become the striker.
  std::vector<Bid> bids = {{1, 9.0, true}, {2, 2.5, true}, {3, 1.0, false}};
  EXPECT_EQ(assign_role(2, bids, 1), RoleBid::ROLE_STRIKER);
}

TEST(RoleAuction, DeterministicTieBreakByPlayerId)
{
  // Equal cost -> lowest player_id wins striker, identically on every robot.
  std::vector<Bid> bids = {{2, 2.0, true}, {3, 2.0, true}};
  EXPECT_EQ(assign_role(2, bids, /*goalie_id=*/0), RoleBid::ROLE_STRIKER);
  EXPECT_EQ(assign_role(3, bids, /*goalie_id=*/0), RoleBid::ROLE_SUPPORTER);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
