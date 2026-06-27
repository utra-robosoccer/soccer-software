# hardware — CAD & PCB (Git LFS)

Mechanical and electrical design artifacts for soccerbot. Binary CAD/PCB files are
tracked with **Git LFS** (see `.gitattributes`) so the robot runtime clone stays
lean (blueprint §11). This folder is owned by the mechanical/electrical teams and
is **not** deployed to robots.

Expected contents (not committed in this scaffold):

```text
cad/        soccerbot.step / .f3d     — frame, neck_pan mount, camera bracket
pcb/        motor_driver.kicad_pcb  — BLDC/FOC driver with CURRENT SENSE
            (current sense is non-negotiable: residual RL needs measured τ — blueprint §9)
bom/        bill_of_materials.csv
```

The PCB must expose, per the RL state-vector requirement (blueprint §9):
joint **position** (encoder), **velocity** (derived), **torque** (current sense),
body **IMU**, and **foot/contact** — published as standard `sensor_msgs` via
micro-ROS so the `ros2_control` boundary is identical in sim and on hardware.
