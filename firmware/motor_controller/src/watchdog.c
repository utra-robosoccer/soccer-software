#include "watchdog.h"

void watchdog_init(Watchdog *wd, uint32_t timeout_us) {
  wd->timeout_us = timeout_us;
  wd->last_rx_us = 0;
  wd->tripped = false;
}

void watchdog_feed(Watchdog *wd, uint32_t now_us) {
  wd->last_rx_us = now_us;
  wd->tripped = false;
}

bool watchdog_expired(Watchdog *wd, uint32_t now_us) {
  // Unsigned subtraction handles timer wrap-around correctly.
  if ((uint32_t)(now_us - wd->last_rx_us) > wd->timeout_us) {
    wd->tripped = true;
  }
  return wd->tripped;
}
