# tools — developer scripts & calibration

Host-side helpers that are **not** deployed to robots.

| Script                   | Purpose                                                                                       |
| ------------------------ | --------------------------------------------------------------------------------------------- |
| `mock_gamecontroller.py` | Broadcast GameController packets on :3838 to drive the stack in sim (no real referee needed). |
| `calibrate_camera.py`    | Produce monocular intrinsics (`fx, fy, cx, cy`) for `soccer_perception`.                      |
| `dev_shell.sh`           | Drop into the dev Docker container with the repo mounted.                                     |

```bash
# Start a 2-robot sim, then put them in PLAYING:
ros2 launch soccer_bringup team.launch.py num_robots:=2
python tools/mock_gamecontroller.py --state playing
```
