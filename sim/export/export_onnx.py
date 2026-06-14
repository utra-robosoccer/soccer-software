"""Export the trained residual policy to ONNX (blueprint §10, step 3).

ONNX is the bridge artifact between the workstation and the robot: the policy is
distro-agnostic and gets converted to a TensorRT engine on the Jetson. The input
name/shape match what ``ResidualRLController`` will feed.

    python export/export_onnx.py --weights runs/residual.npz --out runs/residual.onnx

Then on the Jetson:
    trtexec --onnx=residual.onnx --saveEngine=residual.engine --fp16
"""
from __future__ import annotations

import argparse

import numpy as np

try:
    import torch
    import torch.nn as nn
except Exception as exc:  # pragma: no cover
    raise SystemExit("PyTorch is required to export ONNX: pip install torch") from exc

OBS_DIM = 6
HIDDEN = 16
ACT_DIM = 1
RESIDUAL_LIMIT = 0.20


class PolicyModule(nn.Module):
    """Same architecture as sim/train_residual.py, in torch, for ONNX export."""

    def __init__(self) -> None:
        super().__init__()
        self.fc1 = nn.Linear(OBS_DIM, HIDDEN)
        self.fc2 = nn.Linear(HIDDEN, ACT_DIM)

    def forward(self, obs: "torch.Tensor") -> "torch.Tensor":
        h = torch.tanh(self.fc1(obs))
        return torch.tanh(self.fc2(h)) * RESIDUAL_LIMIT


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="runs/residual.npz")
    ap.add_argument("--out", default="runs/residual.onnx")
    args = ap.parse_args()

    w = np.load(args.weights)
    model = PolicyModule()
    with torch.no_grad():
        model.fc1.weight.copy_(torch.tensor(w["W1"].T, dtype=torch.float32))
        model.fc1.bias.copy_(torch.tensor(w["b1"], dtype=torch.float32))
        model.fc2.weight.copy_(torch.tensor(w["W2"].T, dtype=torch.float32))
        model.fc2.bias.copy_(torch.tensor(w["b2"], dtype=torch.float32))
    model.eval()

    dummy = torch.zeros(1, OBS_DIM)
    torch.onnx.export(
        model, dummy, args.out,
        input_names=["obs"], output_names=["residual"],
        dynamic_axes={"obs": {0: "batch"}, "residual": {0: "batch"}},
        opset_version=17,
    )
    print(f"exported ONNX -> {args.out}")


if __name__ == "__main__":
    main()
