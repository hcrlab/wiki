# How to add a Hokuyo laser to the Turtlebot

This mostly follows the standard ROS tutorial, [Adding a Hokuyo laser to your Turtlebot](http://wiki.ros.org/turtlebot/Tutorials/hydro/Adding%20a%20Hokuyo%20laser%20to%20your%20Turtlebot), but with more specific information.

## Adapter plate 
On our robot, we don't use the 3D printed adapter plate. Instead, we opt to have the laser mounted lower on the robot with tape.

## Setup udev
Follow the tutorial.

## Prepare a catkin environment
This part of the tutorial basically asks you to clone the Turtlebot repository and build it in your workspace. You can try this, but in the past, using the latest code breaks the Turtlebot by introducing lots of other changes involving apps and such. You can instead just skip this step and use `sudo vim` on the system files to get around this.

## Edit your Turtlebot description
Edit turtlebot_library.urdf.xacro with `sudo vim /opt/ros/hydro/share/turtlebot_description/urdf/turtlebot_library.urdf.xacro` or, if you did the previous step, `rosed turtlebot_description turtlebot_library.urdf.xacro`.

If you add the laser using the adapter plate, then their description works. Otherwise, add the following right above the `</robot>` tag:
```xml
<joint name="laser" type="fixed">
  <origin xyz="0.1425 0.00 0.145" rpy="0 0 0" />
  <parent link="base_footprint" />
  <child link="base_laser_link" />
</joint>

<link name="base_laser_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.06" />
    </geometry>
    <material name="Green" />
  </visual>
  <inertial>
    <mass value="0.000001" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
      iyy="0.0001" iyz="0.0"
      izz="0.0001" />
  </inertial>
</link>
```

Edit 3dsensor.launch as described in the tutorial, this will make the Kinect "laser scan" publish to `/kinect_scan` while the Hokuyo publishes to `/scan`. Alternatively, if you include 3dsensor.launch in another launch file, you can simply pass in the arg `scan_topic:=kinect_scan`.

In minimal.launch, add the hokuyo driver:
```xml
<node name="laser_driver" pkg="hokuyo_node" type="hokuyo_node">
  <param name="frame_id" value="base_laser_link" />
</node>
```

## Bringup your new config and check your results with rviz
Running minimal.launch should start the Hokuyo laser. You should see the laser scan on the /scan topic in rviz. Running 3dsensor.launch will start the Kinect. You should see the Kinect "laser scan" on the /kinect_scan topic in rviz. Verify that the data is reasonably well aligned, especially in the range where the Kinect is most accurate (1.8 - 2.4 meters).
