# Cheatsheet

## PR2

### TF

Echo the position of one frame with respect to another:

```bash
rosrun tf tf_echo base_footprint r_gripper_l_finger_tip_frame
```

### Gripper

Open the gripper:

```bash
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "{position: 0.088, max_effort: 100.0}"
```

Close the gripper:

```bash
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "{position: 0.0, max_effort: 100.0}"
```

## Gazebo

Apply a force to a Gazebo body.

```bash
rosservice call /gazebo/apply_body_wrench '{body_name: "pr2::l_forearm_roll_link", reference_frame: "pr2::l_forearm_roll_link", wrench: { force: { x: 0, y: 0, z: -50.0 } }, start_time: 10000000000, duration: 1000000000 }'
```
