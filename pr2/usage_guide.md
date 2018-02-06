Usage guide for the PR2
===

Starting the robot
---
1. Open a terminal and ssh into the robot: `ssh USER@mayarobot-wired`
2. Before using the robot you need to "claim" it so that other people aren't using it simultaneously. Enter the command `robot claim` to do this. If someone else is listed as using the robot, you can steal the robot from them as long as they have stopped using it. 
3. After claiming the robot, you can run `robot start` to start the processes that are prerequisites for moving and sensing. 
4. Once `robot start` completes, you can open a new terminal and enter `setrobot c1` in order to run commands from your terminal on the robot without having to ssh in.

Enabling robot movement
---
1. Press "Start" on the physical wireless run-stop device after `robot start` is complete. This unlocks the motors of the robot so it can move.
2. It is still necessary to reset the motors before you can use them. 
  - In a terminal that is pointed at the robot using `setrobot c1`, run `rosrun rqt_pr2_dashboard rqt_pr2_dashboard`. A separate dashboard window will appear.
  - The dashboard has a gear icon at the top. Click on it and select "Reset Motors" from the dropdown menu. The background of the gear icon should turn from red to green.
  - The dashboard is also useful for checking the status of the robot using the logs and diagnostics displays. 
3. After resetting the motors, the robot is ready to move. Movement commands from your code and from programs on Robot Web Server should work. 

Enabling perception using the kinect
---
You may need to start the head-mounted kinect separately. See the readme titled [Launching the Kinect](https://github.com/hcrlab/wiki/blob/master/kinect/launch.md).

Stopping the robot
---
1. You need to stop and release the robot after using it. 
2. On a terminal ssh'ed into the robot, run `robot stop`. 
3. Press "Stop" on the physical wireless run-stop device.
4. Run `robot release` to release control of the robot.
