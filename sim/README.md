# sim — Isaac Lab training (runs on the workstation, blueprint §10)

Trains the **residual RL policy** that `soccer_control`'s `ResidualRLController`
loads at runtime. Crucially, this is **decoupled from the robot's ROS distro**:
Isaac Lab runs on the x86 RTX workstation and exports an **ONNX** policy; the
robot only ever sees the `.onnx`/`.engine` artifact (blueprint §3.2).

## The sim-to-real recipe (blueprint §10)

1. **Single source of truth** — the URDF/USD in `soccer_description` mirrors the
   sim asset, so the joint the policy controls is the same one on hardware.
2. **Domain randomization** — randomize mass, friction, latency, and sensor noise
   (`tasks/domain_randomization.py`) so the policy is forced to be robust.
3. **Teacher → student distillation** — the teacher trains with privileged sim
   state; the student is distilled to use only the observations the real robot
   has (the exact vector the controller feeds: `[q, qd, q_ref, qd_ref, gyro_z, command]`).
4. **System identification** — log real motor responses, fit sim params, feed back
   into DR. (Do **not** fine-tune RL live on the robot — blueprint §10.)

## Layout

```text
tasks/      soccerbot_reach_env.py (Isaac Lab task + numpy fallback) · domain_randomization.py
export/     export_onnx.py  (policy -> ONNX, the artifact the controller loads)
assets/     USD models (Git LFS) — mirrors soccer_description URDF
train_residual.py   end-to-end: train (fallback ES) -> save weights
```

## Quick start

```bash
pip install -r requirements.txt

# Train the residual policy. With Isaac Lab installed it uses the GPU env;
# otherwise it falls back to a tiny CPU env so the pipeline is runnable anywhere.
python train_residual.py --iterations 200 --out runs/residual.npz

# Export to ONNX for deployment (consumed by ResidualRLController.policy_path).
python export/export_onnx.py --weights runs/residual.npz --out runs/residual.onnx
```

The exported `residual.onnx` is the bridge artifact: CI publishes it alongside
the runtime image, and the controller hard-clamps its output so a bad policy can
never destabilize the joint (blueprint §4).
