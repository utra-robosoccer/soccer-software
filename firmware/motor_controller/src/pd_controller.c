#include "pd_controller.h"

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float pd_compute_torque(const PdConfig *cfg, const PdCommand *cmd,
                        float q_meas, float qd_meas) {
  // Use per-command gains when provided, else the configured defaults.
  const float kp = (cmd->kp > 0.0f) ? cmd->kp : cfg->kp;
  const float kd = (cmd->kd > 0.0f) ? cmd->kd : cfg->kd;

  // Respect the mechanical joint limits: never command past a hard stop.
  const float q_des = clampf(cmd->q_des, cfg->q_min, cfg->q_max);

  // MIT-style impedance law: τ = kp·(q*-q) + kd·(qd*-qd) + τ_ff.
  const float tau = kp * (q_des - q_meas) + kd * (cmd->qd_des - qd_meas) + cmd->tau_ff;
  return clampf(tau, -cfg->tau_max, cfg->tau_max);
}
