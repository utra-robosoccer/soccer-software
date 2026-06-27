# sim/assets — heavy simulation assets (Git LFS)

USD models for Isaac Sim live here and are tracked with **Git LFS** (see
`.gitattributes`) so the robot runtime clone stays lean (blueprint §11).

The USD geometry **must mirror** `ros2_ws/src/soccer_description/urdf/soccerbot.urdf.xacro`
— the URDF is the single source of truth for kinematics, and the USD is its
render/physics twin. Keeping them in sync is what makes the `ros2_control`
boundary identical in sim and on hardware.

Expected files (not committed in this scaffold):

```text
soccerbot.usd      # Soccerbot: base + neck_pan + camera + imu (mirrors the URDF)
field.usd          # the 6x4 m field used by the perception/localization demo
```
