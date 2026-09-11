# EN3563 Mini Project — GP7 Pick-and-Place (Gazebo simulation)

Pick-and-place of Box B onto Box A with a Yaskawa Motoman GP7, fully in simulation.

## Structure

```
6DoF-Robot-Manipulator-Simulation/
├── report/                 D-H table, FK/IK derivations, writeup
├── media/                  screen-recorded simulation video
└── src/
    ├── gp7_description/    URDF/xacro, meshes, worlds, ros2_control + Gazebo bring-up
    └── gp7_pick_place/     FK/IK, trajectory planning, velocity control, mission node
```

## Setup

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Verify the model in RViz (no physics, just geometry/TF):
```bash
ros2 launch gp7_description display.launch.py
```

Bring up the full Gazebo simulation (Box A, Box B, ros2_control):
```bash
ros2 launch gp7_description gazebo.launch.py
```

## Acknowledgment

`gp7_description`'s link geometry, joint origins/axes/limits, and meshes are
ported from the ROS-Industrial project's `motoman_gp7_support` package
(github.com/ros-industrial/motoman, `kinetic-devel` branch, BSD licensed).
Per-link mass/inertia (not present in the source, which only needed
visual/collision geometry for ROS 1 MoveIt use) was derived from real mesh
volume, scaled so total mass matches Yaskawa's published 34 kg product mass.
The ROS 1 → ROS 2 packaging, the simplified gripper end effector, and the
Gazebo/ros2_control integration are original to this project.

## Status

- [x] GP7 model ported to ROS 2, gripper attached, verified in RViz
- [x] Gazebo scene (Box A, Box B), ros2_control, both controllers verified with live commands
- [ ] D-H parameter table + coordinate frame diagram (report)
- [ ] Forward kinematics
- [ ] Inverse kinematics
- [ ] Trajectory planning + velocity profile
- [ ] Pick-and-place demo video
