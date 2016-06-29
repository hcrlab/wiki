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
