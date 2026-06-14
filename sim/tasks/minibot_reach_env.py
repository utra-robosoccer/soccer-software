"""MiniBot "reach/track" residual-RL task.

The policy learns the **bounded residual** Δq the controller adds on top of the
MPC reference so the neck_pan joint tracks a commanded bearing despite the
randomized dynamics. The observation is EXACTLY what ``ResidualRLController``
feeds at runtime, so the trained policy drops straight in:

    obs = [q, qd, q_ref, qd_ref, gyro_z, command_bearing]
    act = Δq  (clamped to ±residual_limit by the controller)

Two implementations share one observation/return contract:

* ``MinibotReachEnvCfg`` — an Isaac Lab ``ManagerBasedRLEnvCfg`` skeleton for the
  real GPU-parallel training run (the production path).
* ``NumpyReachEnv``     — a tiny dependency-light CPU env so ``train_residual.py``
  and the whole export pipeline are runnable on any laptop without Isaac.
"""
from __future__ import annotations

import numpy as np

from sim.tasks.domain_randomization import DomainRandomization

OBS_DIM = 6
ACT_DIM = 1
RESIDUAL_LIMIT = 0.20  # must match controllers.yaml residual_limit_rad


# ──────────────────────────────────────────────────────────────────────────────
# Production path: Isaac Lab task configuration (skeleton).
# ──────────────────────────────────────────────────────────────────────────────
class MinibotReachEnvCfg:
    """Isaac Lab ManagerBasedRLEnvCfg for the residual-tracking task.

    Filled out against the installed Isaac Lab API on the workstation. Mirrors the
    URDF in soccer_description (single source of truth) and applies the same DR as
    the fallback so teacher/student see identical observation semantics.
    """

    episode_length_s = 4.0
    decimation = 4          # policy at ~50 Hz when sim steps at 200 Hz
    dr = DomainRandomization()

    # observations -> OBS_DIM, actions -> ACT_DIM (residual on neck_pan)
    # reward = -|q - q_cmd| - 0.01*|Δq| - 0.001*|qd|   (track with a small, smooth residual)
    # See Isaac Lab managers (ObservationsCfg / RewardsCfg / EventCfg) when wiring.


# ──────────────────────────────────────────────────────────────────────────────
# Fallback path: a tiny CPU env (runnable anywhere).
# ──────────────────────────────────────────────────────────────────────────────
class NumpyReachEnv:
    """Single-joint, second-order dynamics with domain randomization."""

    def __init__(self, seed: int = 0, dt: float = 0.02) -> None:
        self.rng = np.random.default_rng(seed)
        self.dt = dt
        self.dr = DomainRandomization()
        self.reset()

    def reset(self) -> np.ndarray:
        self.cmd = float(self.rng.uniform(-1.2, 1.2))   # target bearing to hold
        # Start AT the target so the dominant control problem is rejecting the
        # unmodeled disturbance below — which is exactly what the bounded residual
        # RL is for (blueprint §4). Without a residual the joint drifts off target
        # by `offset`; the policy must learn to hold against it.
        self.q = self.cmd
        self.qd = 0.0
        self.t = 0.0
        self.params = self.dr.sample(self.rng)          # randomize this episode
        # Unmodeled steady-state offset the MPC does NOT account for (a miscalibrated
        # zero / constant load), within the residual limit so it is cancelable.
        self.offset = float(self.rng.uniform(0.08, 0.14))
        return self._obs(q_ref=self.cmd, qd_ref=0.0)

    def _obs(self, q_ref: float, qd_ref: float) -> np.ndarray:
        gyro = self.qd + self.rng.normal(0, self.dr.gyro_noise_std)
        q_noisy = self.q + self.rng.normal(0, self.dr.encoder_noise_std)
        return np.array([q_noisy, self.qd, q_ref, qd_ref, gyro, self.cmd],
                        dtype=np.float32)

    def step(self, residual: float):
        # The MPC reference is a simple clamped move toward the command; the policy
        # supplies the residual on top (exactly the runtime decomposition).
        q_ref = np.clip(self.cmd, -1.57, 1.57)
        qd_ref = 0.0
        residual = float(np.clip(residual, -RESIDUAL_LIMIT, RESIDUAL_LIMIT))
        q_target = q_ref + residual

        # Integrate the stiff second-order dynamics with a 1 kHz inner loop (20
        # substeps per 50 Hz control step) — both numerically stable AND faithful
        # to the real MCU PD loop running under the policy.
        p = self.params
        kp, kd = 120.0, 8.0
        substeps = 20
        h = self.dt / substeps
        for _ in range(substeps):
            # The plant carries an unmodeled offset; the policy's residual must
            # cancel it to drive the true joint angle q to the command.
            plant_target = q_target - self.offset
            torque = (kp * (plant_target - self.q) - kd * self.qd) * p["torque_scale"]
            accel = (torque - p["damping"] * self.qd) / p["inertia"]
            self.qd = float(np.clip(self.qd + accel * h, -6.0, 6.0))
            self.q = float(np.clip(self.q + self.qd * h, -1.57, 1.57))
        self.t += self.dt

        err = abs(self.q - self.cmd)
        reward = -err - 0.01 * abs(residual) - 0.001 * abs(self.qd)
        done = self.t >= 4.0
        return self._obs(q_ref, qd_ref), reward, done, {"err": err}
