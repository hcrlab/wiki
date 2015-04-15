# How to calibrate the Kinect

## Intrinsics
Calibrate the intrinsics using this tutorial: [Intrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). Save the files to /etc/ros/camera_info, and make sure the files are readable by all.

## Extrinsics
**Note**: extrinsic calibration doesn't seem to exist at all in Groovy or Hydro. We have filed a ticket with Clearpath, but in the meantime, there doesn't seem to be a way to do this.

Next, try following this tutorial: [Extrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/ExtrinsicCalibration). There is one hiccup, which is that [camera_pose_calibration](http://wiki.ros.org/camera_pose_calibration) doesn't exist in Hydro. It's also not installable from Ubuntu repos in Groovy. So, you need to clone it from the [Github repo](https://github.com/ros-perception/camera_pose) into your rosbuild workspace.

The next problem is that camera_pose won't build properly. To fix this, edit `camera_pose_toolkits/manifest.xml` and add this line:
```xml
<depend package="rosbag"/>
```

Now the package should build.

## PR2 calibration
The [PR2 calibration](http://wiki.ros.org/pr2_calibration) tutorial works pretty well. The only thing is that you should run `/etc/ros/distro/openni_head.launch` to start the Kinect.
