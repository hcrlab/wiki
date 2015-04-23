# Cheatsheet

## PR2

### TF

Echo the position of one frame with respect to another:

```
rosrun tf tf_echo base_footprint r_gripper_l_finger_tip_frame
```

### Gripper

Open the gripper:

```
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "{position: 0.088, max_effort: 100.0}"
```

Close the gripper:

```
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "{position: 0.0, max_effort: 100.0}"
```

