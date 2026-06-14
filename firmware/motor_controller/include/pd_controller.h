// 1 kHz PD / MIT-impedance joint control (L0, blueprint §4, §9).
//
// This is the ONLY loop that runs at 1 kHz, and it lives on the MCU — NOT on the
// Jetson (which is not hard-real-time). It tracks the position/velocity target
// the Jetson streams, with a feed-forward torque term, and clamps the output.
#ifndef MINIBOT_PD_CONTROLLER_H_
#define MINIBOT_PD_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp;        // default position gain (used if command kp==0)
  float kd;        // default velocity gain
  float tau_max;   // hard torque clamp (N·m)
  float q_min;     // joint lower limit (rad)
  float q_max;     // joint upper limit (rad)
} PdConfig;

typedef struct {
  float q_des, qd_des, tau_ff;  // latest command
  float kp, kd;                 // active gains
} PdCommand;

// Compute the commanded torque for one 1 kHz tick. Pure function => host-testable.
float pd_compute_torque(const PdConfig *cfg, const PdCommand *cmd,
                        float q_meas, float qd_meas);

#ifdef __cplusplus
}
#endif

#endif  // MINIBOT_PD_CONTROLLER_H_
