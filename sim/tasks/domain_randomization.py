"""Domain randomization config (blueprint §10, step 1).

Randomizing the physics + sensors each episode forces the policy to be robust
instead of overfitting one perfect sim, which is what lets it transfer to the
real MiniBot. These ranges are the knobs system-identification later tightens to
match measured hardware.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DomainRandomization:
    # Physical parameters of the single neck_pan joint.
    inertia_range: tuple = (0.006, 0.016)     # kg·m²
    damping_range: tuple = (0.02, 0.10)       # N·m·s/rad
    friction_range: tuple = (0.0, 0.05)       # N·m
    # Actuation realism.
    torque_scale_range: tuple = (0.85, 1.15)  # gear/efficiency mismatch
    latency_steps_range: tuple = (0, 3)       # command/observation delay (ticks)
    # Sensor noise (observation corruption).
    encoder_noise_std: float = 0.005          # rad
    gyro_noise_std: float = 0.01              # rad/s

    def sample(self, rng):
        def u(r):
            return rng.uniform(*r)
        return {
            "inertia": u(self.inertia_range),
            "damping": u(self.damping_range),
            "friction": u(self.friction_range),
            "torque_scale": u(self.torque_scale_range),
            "latency_steps": int(rng.integers(self.latency_steps_range[0],
                                              self.latency_steps_range[1] + 1)),
        }
