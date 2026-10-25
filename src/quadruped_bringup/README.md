# quadruped_bringup

Launch files, worlds and configuration for the Go2 in Gazebo.

```bash
ros2 launch quadruped_bringup sim.launch.py rviz:=true       # Gazebo GUI + RViz
ros2 launch quadruped_bringup sim.launch.py headless:=true   # server only
ros2 launch quadruped_bringup sim.launch.py joy:=true        # + PS5 gamepad teleop
ros2 service call /stand_up std_srvs/srv/Trigger
ros2 service call /lie_down std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # keyboard teleop (own terminal)
```

Launch arguments: `rviz`, `headless`, `cheat` (default `true`: the PlanarBodyDriver plugin
pushes the body along `/cmd_vel`; `false` for real walking later), `joy`, `world`.

## PS5 controller (`config/teleop_ps5.yaml`)

Connect the DualSense by USB or Bluetooth on the host; it appears in the container.

| Input | Action |
|---|---|
| hold **L1** | dead-man switch: nothing moves without it |
| left stick up/down | forward / backward |
| left stick left/right | turn |
| right stick left/right | sideways |
| hold **R1** too | turbo |
| **Cross** / **Circle** | stand up / lie down |

Indices follow SDL's standard gamepad layout (`joy`'s `game_controller_node`).
If something is mapped wrong, check `ros2 topic echo /joy` and edit the YAML.

## Topics

| Topic | What |
|---|---|
| `/cmd_vel` | velocity command (Twist) -> Gazebo PlanarBodyDriver and the leg animation |
| `/odom` + TF `odom -> base_link` | noisy dead-reckoning odometry (`noisy_odom_node`), drifts |
| `/ground_truth/odom` | perfect pose from Gazebo, for comparison only |

## Config files

- `config/controllers.yaml`: ros2_control (500 Hz): `joint_state_broadcaster` +
  `forward_position_controller` (12 joints, order = command array order).
- `config/bridge.yaml`: Gazebo <-> ROS topics.
- `worlds/empty.sdf`: ground plane, 1 kHz DART physics, sensor/IMU/contact systems.
