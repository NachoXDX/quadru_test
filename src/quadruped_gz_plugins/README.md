# quadruped_gz_plugins

Gazebo (gz-sim) system plugins for the quadruped simulation.

## PlanarBodyDriver

"Cheat" locomotion: applies a horizontal force and a yaw torque to the model's
canonical link, proportional to the error between the commanded twist (gz topic,
`gz.msgs.Twist`, bridged from ROS `/cmd_vel`) and the measured velocity. Height,
roll and pitch stay physical, so the robot can still fall, sit and wobble.

Unlike gz `VelocityControl`, which overrides the whole body velocity every step
(a robot spawned in the air with it sinks at ~1 cm/s instead of falling).

Parameters (SDF, all optional): `topic`, `linear_gain` (N per m/s), `angular_gain`
(N*m per rad/s), `max_force`, `max_torque`, `cmd_timeout`. See the header comment in
`src/planar_body_driver.cpp`; the Go2 sets them in `go2_description/urdf/go2.gazebo.xacro`.
