# A single robot's bid for a role in the decentralized role auction
# (blueprint §6). Every robot broadcasts bids; each independently computes the
# same highest-utility assignment, so there is NO master and dropouts trigger an
# automatic re-bid.

uint8 ROLE_UNASSIGNED = 0
uint8 ROLE_STRIKER = 1
uint8 ROLE_SUPPORTER = 2
uint8 ROLE_GOALIE = 3

uint8 role
# Lower cost = stronger claim. Typically distance-to-ball (metres).
float64 cost
