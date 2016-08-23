# Launch file for the Kinect

After experimentation between using openni and also with publishing TFs, here is the final Kinect launch file that works best on the PR2:

```xml
<launch>
  <arg name="kinect_camera_name" default="head_mount_kinect" />

  <!-- Turn on the Kinect -->
  <include file="$(find freenect_launch)/launch/freenect.launch">
    <arg name="depth_registration" value="true"/>
    <arg name="camera" value="$(arg kinect_camera_name)"/>
    <arg name="depth_frame_id" value="head_mount_kinect_ir_optical_frame" />
    <arg name="publish_tf" value="false"/>
  </include>
</launch>
```
