# go2_description

Unitree Go2 model for ROS 2 (xacro + meshes).

`urdf/go2_body.xacro` and `meshes/` are adapted from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) `robots/go2_description`
(BSD-3-Clause, see `LICENSE`). Changes: inline materials removed, mesh paths moved to `meshes/`,
a massless `base_link` root added (KDL and Nav2/robot_localization conventions).

```bash
ros2 launch go2_description display.launch.py          # sliders for every joint
```

Joints per leg (`FL`, `FR`, `RL`, `RR`): `<leg>_hip_joint` (abduction, x axis),
`<leg>_thigh_joint` and `<leg>_calf_joint` (pitch, y axis). Thigh and calf are 0.213 m long.
