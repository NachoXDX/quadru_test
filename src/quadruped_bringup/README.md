# quadruped_bringup

Launch files, worlds and configuration for the Go2 in Gazebo.

```bash
ros2 launch quadruped_bringup sim.launch.py              # Gazebo GUI; add rviz:=true for RViz
ros2 launch quadruped_bringup sim.launch.py headless:=true   # server only
ros2 service call /stand_up std_srvs/srv/Trigger
ros2 service call /lie_down std_srvs/srv/Trigger
ros2 control list_controllers
```

- `config/controllers.yaml`: ros2_control (500 Hz): `joint_state_broadcaster` +
  `forward_position_controller` (12 joints, order = command array order).
- `config/bridge.yaml`: Gazebo <-> ROS topics (`/clock` for now).
- `worlds/empty.sdf`: ground plane, 1 kHz DART physics, sensor/IMU/contact systems.
