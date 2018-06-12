# `robot_calibration` approach

- [robot_calibration](https://wiki.ros.org/robot_calibration)
- [pr2_robot_calibration](https://github.com/jstnhuang/pr2_robot_calibration)

Take a look at the [robot_calibration](https://wiki.ros.org/robot_calibration) package, which is a robot-agnostic calibration package.
To use `robot_calibration`, you must create launch files and config files specific to the PR2.
An example of these can be found in [pr2_robot_calibration](https://github.com/jstnhuang/pr2_robot_calibration).
This package was successfully used to calibrate our PR2 as of June 2018.
However, by the time you read this, it's likely that the API will have changed and additional changes are necessary.
When in doubt, use [fetch_calibration](http://wiki.ros.org/fetch_calibration) as a template.

# After reboots
When the robot reboots, it will run a calibration routine to compute certain joint offsets
You should clear the space around the robot before rebooting, because it will move its arms while calibrating.
The torso will raise up, then the arms will move, then the torso will move all the way down.
The head will move a bit, and finally, it will try to move the casters.
However, we have noticed that the casters tend to get stuck (perhaps due to the carpet in the lab), so the calibration routine will hang for a while saying, "caster joint is taking a long time to calibrate and may need human help."
Just wait for a few minutes and it will eventually finish.

Note: you can force this calibration routine to run without restarting the entire robot by opening the PR2 dashboard and disabling all breakers, then re-enabling them.

# Camera calibration
- [Kinect calibration](https://github.com/hcrlab/wiki/blob/master/kinect/calibration/README.md)
- [ROS camera calibration](http://wiki.ros.org/camera_calibration)

# Full calibration
**Note**: These instructions are for posterity.
Our PR2's laser does not work, nor do we have the space to capture the "far" poses of the calibration.
As a result, these steps are unlikely to work.

- [PR2 calibration](http://wiki.ros.org/pr2_calibration/Tutorials/Calibrating%20the%20PR2)

Use the above tutorial with a few changes.
If capture_data.launch complains about being unable to parse the URDF, it is probably using the default URDF parser.
The PR2 has a customized version of the URDF parser.
You can make sure it is being used by running the following command:
```bash
export PYTHONPATH=`rospack find pr2_calibration_launch`/pr2_urdf_parser_py:$PYTHONPATH
```
And then running capture_data.launch in the same terminal window.

Also, be sure to start up the kinect, and map the camera name to /kinect_head:
```xml
<launch>
  <arg name="kinect_camera_name" value="kinect_head" />

  <!-- Turn on the Kinect -->
  <include file="$(find freenect_launch)/launch/freenect.launch">
    <arg name="depth_registration" value="true"/>
    <arg name="camera" value="$(arg kinect_camera_name)"/>
    <arg name="rgb_frame_id" value="head_mount_kinect_rgb_optical_frame" />
    <arg name="depth_frame_id" value="head_mount_kinect_ir_optical_frame" />
    <arg name="publish_tf" value="false"/>
  </include>
</launch>
```
