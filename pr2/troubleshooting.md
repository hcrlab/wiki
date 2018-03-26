# Troubleshooting

- [Clearpath FAQ on common issues](https://pr2s.clearpathrobotics.com/wiki/FAQ)
- [Troubleshooting wireless problems](wireless_problems.md)

### No data from runstop
Sometimes the robot will report no data from the runstop.
I.e., even with the runstop on, the robot will not report that it's on.
In `rqt_pr2_dashboard`, it will report "stale data."
The dashboard might also report issues with the power board.
After checking, run `robot stop -f` and then run `sudo pr2-systemcheck` and see if it reports an error with the power board.
If so, the only solution we've seen so far is the power-cycle the robot.

### The robot won't move
* Is it runstopped?
* If using `rosrun` or `roslaunch` from your desktop, make sure you have first run `setrobot c1` so you're talking to roscore on the robot.

### `robot start` isn't working
* Do you have passwordless ssh set up from c1 to c2? This is required for the script to run.
* Make sure there is no mismatch between groovy and hydro in your environment. It's possible to have your environment in an inconsistent state where it's partially set to both, which breaks `robot start`. If unsure, follow these directions to [fully set the robot to groovy or hydro](https://github.com/hcrlab/wiki/blob/master/pr2/switching_robot_to_groovy.md).

### Power-cycling the PR2
* `ssh` into robot and run `sudo pr2-shutdown`. Your connection will be closed and you hear beeping sounds.
* Turn the power off by pressing the red switch at the bottom of the robot's base.
* Wait for a minute before turning the robot back on. In the meantime, make sure there is enough space around the robot for the calibration dance when it is turned back on (the arms are stretched during calibration). Also, ensure the physical run-stop (the red push botton in the middle of the robot's torso) is out. Press START on the wireless run-stop as well.
* Turn the power on using the same switch at the bottom of the base. You should hear a series of beeps before the calibration dance.

If you did not see the calibration dance:
* `ssh` to robot and start it (first `robot claim` then `robot start`).
* On a separate terminal run PR2 dashboard `rosrun rqt_pr2_dashboard rqt_pr2_dashboard` (first make sure you properly set `ROS_MASTER_URI`, e.g. by running `setrobot c1`).
* Reset the motors if they are halted from the dahsboard.
