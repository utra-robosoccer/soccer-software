// Host-side unit tests for the 1 kHz control math + watchdog. No framework: any
// failed check returns non-zero so ctest marks the test failed.
#include "pd_controller.h"
#include "watchdog.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      printf("FAIL: %s (line %d)\n", #cond, __LINE__);                 \
      ++failures;                                                      \
    }                                                                  \
  } while (0)

static void test_pd_drives_toward_target(void) {
  PdConfig cfg = {.kp = 100.0f, .kd = 5.0f, .tau_max = 3.0f,
                  .q_min = -1.57f, .q_max = 1.57f};
  PdCommand cmd = {.q_des = 0.5f, .qd_des = 0.0f, .tau_ff = 0.0f, .kp = 0, .kd = 0};
  // Below target, at rest -> positive torque (push up toward target).
  float tau = pd_compute_torque(&cfg, &cmd, 0.0f, 0.0f);
  CHECK(tau > 0.0f);
}

static void test_pd_clamps_to_tau_max(void) {
  PdConfig cfg = {.kp = 1000.0f, .kd = 0.0f, .tau_max = 3.0f,
                  .q_min = -10.0f, .q_max = 10.0f};
  PdCommand cmd = {.q_des = 5.0f, .qd_des = 0.0f, .tau_ff = 0.0f, .kp = 0, .kd = 0};
  float tau = pd_compute_torque(&cfg, &cmd, 0.0f, 0.0f);
  CHECK(fabsf(tau) <= cfg.tau_max + 1e-6f);
  CHECK(tau == cfg.tau_max);
}

static void test_pd_respects_joint_limits(void) {
  PdConfig cfg = {.kp = 10.0f, .kd = 0.0f, .tau_max = 100.0f,
                  .q_min = -1.0f, .q_max = 1.0f};
  // Command beyond the upper limit is clamped, so torque matches limit, not 5.0.
  PdCommand cmd = {.q_des = 5.0f, .qd_des = 0.0f, .tau_ff = 0.0f, .kp = 0, .kd = 0};
  float tau = pd_compute_torque(&cfg, &cmd, 0.0f, 0.0f);
  CHECK(fabsf(tau - 10.0f * 1.0f) < 1e-4f);  // kp * (q_max - q)
}

static void test_watchdog_trips_after_timeout(void) {
  Watchdog wd;
  watchdog_init(&wd, /*timeout_us=*/1000u);
  watchdog_feed(&wd, 10000u);
  CHECK(!watchdog_expired(&wd, 10500u));  // within timeout
  CHECK(watchdog_expired(&wd, 12000u));   // silent too long -> tripped
}

int main(void) {
  test_pd_drives_toward_target();
  test_pd_clamps_to_tau_max();
  test_pd_respects_joint_limits();
  test_watchdog_trips_after_timeout();
  if (failures == 0) {
    printf("all firmware control tests passed\n");
  }
  return failures;
}
