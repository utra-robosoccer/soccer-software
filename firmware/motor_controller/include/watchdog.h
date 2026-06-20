// Bus-silence watchdog (L0 safety, blueprint §0, §9).
//
// If the Jetson process dies or the link drops, the MCU must independently zero
// torque — it does not wait for permission. This is the last line of safety.
#ifndef MINIBOT_WATCHDOG_H_
#define MINIBOT_WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t timeout_us;     // max allowed silence before tripping
  uint32_t last_rx_us;     // timestamp of last valid command
  bool tripped;
} Watchdog;

void watchdog_init(Watchdog *wd, uint32_t timeout_us);

// Call on every valid, CRC-checked command.
void watchdog_feed(Watchdog *wd, uint32_t now_us);

// Returns true if the bus has gone silent past the timeout (=> zero torque).
bool watchdog_expired(Watchdog *wd, uint32_t now_us);

#ifdef __cplusplus
}
#endif

#endif  // MINIBOT_WATCHDOG_H_
