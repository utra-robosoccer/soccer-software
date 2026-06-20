// MiniBot motor-controller firmware — the 1 kHz real-time loop (L0).
//
// Target: STM32/Teensy MCU on the firmware team's PCB. This is the ONLY part of
// the stack that is hard-real-time. It is intentionally tiny: receive an
// impedance command from the Jetson, run the PD law at 1 kHz, drive the motor,
// feed the watchdog, and stream measured state back. If the bus goes silent the
// watchdog zeroes torque independently of the Jetson (blueprint §0, §9).
//
// The board-support functions (hal_*, microros_*) are provided by the firmware
// team's BSP; this file is the portable control structure. It is compiled only
// for the MCU target (BUILD_MCU_FIRMWARE), never on the host CI.
#include "pd_controller.h"
#include "protocol.h"
#include "watchdog.h"

#include <string.h>

// ── Provided by the board support package (CAN/serial, timer, motor driver) ──
extern uint32_t hal_micros(void);
extern int hal_recv_motor_cmd(MotorCmd *out);          // non-blocking; 1 if frame ready
extern void hal_send_motor_state(const MotorState *s);
extern void hal_apply_torque(float tau_nm);
extern float hal_read_position(void);
extern float hal_read_velocity(void);
extern float hal_read_current_torque(void);            // current-sense -> τ (RL needs this!)
extern float hal_read_temperature(void);
extern void microros_spin_once(void);

static PdConfig g_cfg = {
    .kp = 120.0f, .kd = 8.0f, .tau_max = 3.0f, .q_min = -1.57f, .q_max = 1.57f};
static PdCommand g_cmd = {0};
static Watchdog g_wd;

// Called by the 1 kHz hardware timer ISR.
void control_tick_1khz(void) {
  const uint32_t now = hal_micros();

  // 1. Ingest the latest command if one arrived (CRC-checked).
  MotorCmd rx;
  if (hal_recv_motor_cmd(&rx)) {
    const uint16_t want = minibot_crc16((const uint8_t *)&rx,
                                        sizeof(MotorCmd) - sizeof(uint16_t));
    if (want == rx.crc) {
      g_cmd.q_des = rx.q;
      g_cmd.qd_des = rx.qd;
      g_cmd.tau_ff = rx.tau_ff;
      g_cmd.kp = rx.kp;
      g_cmd.kd = rx.kd;
      watchdog_feed(&g_wd, now);
    }
  }

  // 2. Read sensors.
  const float q = hal_read_position();
  const float qd = hal_read_velocity();

  // 3. Safety: bus silent -> zero torque (independent of the Jetson).
  float tau;
  if (watchdog_expired(&g_wd, now)) {
    tau = 0.0f;
  } else {
    tau = pd_compute_torque(&g_cfg, &g_cmd, q, qd);
  }
  hal_apply_torque(tau);

  // 4. Stream measured state back to the Jetson.
  MotorState st;
  st.pos = q;
  st.vel = qd;
  st.eff = hal_read_current_torque();
  st.temp = hal_read_temperature();
  st.ts_us = now;
  st.crc = minibot_crc16((const uint8_t *)&st, sizeof(MotorState) - sizeof(uint16_t));
  hal_send_motor_state(&st);
}

int main(void) {
  watchdog_init(&g_wd, /*timeout_us=*/50000u);  // 50 ms of silence => safe-halt
  // hal_timer_start_1khz(control_tick_1khz);   // wire the ISR (BSP)
  for (;;) {
    microros_spin_once();  // background comms; the control loop runs in the ISR
  }
  return 0;
}
