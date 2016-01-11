### The robot won't move
* Is it runstopped?
* If using `rosrun` or `roslaunch` from your desktop, make sure you have first run `setrobot c1` so you're talking to roscore on the robot.

### `robot start` isn't working
* Do you have passwordless ssh set up from c1 to c2? This is required for the script to run.
* Make sure there is no mismatch between groovy and hydro in your environment. It's possible to have your environment in an inconsistent state where it's partially set to both, which breaks `robot start`. If unsure, follow these directions to [fully set the robot to groovy or hydro](https://github.com/hcrlab/wiki/blob/master/pr2/switching_robot_to_groovy.md).
