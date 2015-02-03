# How to run leg_detector

These instructions describe how to use the leg detector for the Turtlebot on Hydro. But, the process should be similar for other robots.

## Install
Install the people stack: `sudo apt-get install ros-hydro-people`
The leg_detector package will be installed to `/opt/ros/hydro/share/leg_detector`, you can also browse its [source on Github](https://github.com/wg-perception/people/tree/hydro-devel/leg_detector).

The leg_detector package's [leg_detector.launch](https://github.com/wg-perception/people/blob/hydro-devel/leg_detector/launch/leg_detector.launch) file hard-codes a `scan:=base_scan` remapping argument. For the Turtlebot, the laser scan is just on the /scan topic, so you can get rid of this. Create new launch file called turtlebot.launch which gets rid of the `scan:=base_scan` argument.

## Run

### On the Turtlebot
Start the Turtlebot:
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch
```

Use the extended Kalman filter to get /odom_combined:
```
rosrun robot_pose_ekf robot_pose_ekf
```

Launch the leg detector:
```
roslaunch leg_detector turtlebot.launch
```

### Visualize the data
On your workstation, run rviz. Set your global fixed frame to /odom_combined. Add a marker display, and set the update topic to /visualization_marker. Add a laser scan display, with the update topic set to /scan.

## Troubleshooting
### Verify laser scan
In rviz, verify that the laser scan shows up, and that it makes sense. Run `rosnode info leg_detector` and verify that it's subscribed to the right laser scan topic.

### TF errors
As with most TF errors, you may want to synchronize the clocks on the robot and the workstation by running this command on both: `sudo ntpupdate ntp.ubuntu.com`.
