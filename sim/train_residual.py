"""Train the residual policy (blueprint §10).

The production run is GPU-parallel PPO in Isaac Lab on ``MinibotReachEnvCfg``.
For portability this script ships a tiny **evolution-strategies** trainer on the
CPU fallback env so the *entire* train -> export -> deploy pipeline is runnable
without Isaac or a GPU. Either way the output is a small MLP with the same
observation/return contract the controller expects, saved as ``.npz`` weights.

    python train_residual.py --iterations 200 --out runs/residual.npz
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from sim.tasks.minibot_reach_env import ACT_DIM, OBS_DIM, RESIDUAL_LIMIT, NumpyReachEnv

HIDDEN = 16


class MlpPolicy:
    """obs(6) -> tanh(16) -> tanh(1) * residual_limit."""

    def __init__(self, rng: np.random.Generator) -> None:
        self.W1 = rng.normal(0, 0.3, (OBS_DIM, HIDDEN))
        self.b1 = np.zeros(HIDDEN)
        self.W2 = rng.normal(0, 0.3, (HIDDEN, ACT_DIM))
        self.b2 = np.zeros(ACT_DIM)

    def act(self, obs: np.ndarray) -> float:
        h = np.tanh(obs @ self.W1 + self.b1)
        out = np.tanh(h @ self.W2 + self.b2)
        return float(out[0]) * RESIDUAL_LIMIT

    # Flatten / unflatten for evolution strategies.
    def get(self) -> np.ndarray:
        return np.concatenate([self.W1.ravel(), self.b1, self.W2.ravel(), self.b2])

    def set(self, v: np.ndarray) -> None:
        i = 0
        for arr in (self.W1, self.b1, self.W2, self.b2):
            n = arr.size
            arr[...] = v[i:i + n].reshape(arr.shape)
            i += n

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2)


def rollout(policy: MlpPolicy, env: NumpyReachEnv, episodes: int = 4,
            seed: int | None = None) -> float:
    # Common random numbers: reseeding the env so every candidate in a generation
    # faces the SAME targets + dynamics makes the ES fitness comparison fair.
    if seed is not None:
        env.rng = np.random.default_rng(seed)
    total = 0.0
    for _ in range(episodes):
        obs = env.reset()
        done = False
        while not done:
            obs, reward, done, _ = env.step(policy.act(obs))
            total += reward
    return total / episodes


def train(iterations: int, out: str, seed: int = 0) -> None:
    rng = np.random.default_rng(seed)
    env = NumpyReachEnv(seed=seed)
    policy = MlpPolicy(rng)
    theta = policy.get()
    n_params = theta.size
    pop, sigma, lr = 32, 0.08, 0.03

    for it in range(iterations):
        eval_seed = int(rng.integers(1_000_000_000))  # shared across the population
        noise = rng.normal(0, 1, (pop, n_params))
        returns = np.zeros(pop)
        for k in range(pop):
            policy.set(theta + sigma * noise[k])
            returns[k] = rollout(policy, env, seed=eval_seed)
        # Rank-normalize and take an ES gradient step.
        adv = (returns - returns.mean()) / (returns.std() + 1e-8)
        theta = theta + lr / (pop * sigma) * (noise.T @ adv)
        if it % 10 == 0:
            policy.set(theta)
            print(f"iter {it:4d}  return {rollout(policy, env, seed=seed):.3f}")

    policy.set(theta)
    policy.save(out)
    print(f"saved policy weights -> {out}  "
          f"(final return {rollout(policy, env, seed=seed):.3f})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iterations", type=int, default=200)
    ap.add_argument("--out", default="runs/residual.npz")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    train(args.iterations, args.out, args.seed)
