# How to calibrate the Kinect

## Intrinsics
Calibrate the intrinsics using this tutorial: [Intrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). Save the files to /etc/ros/camera_info, and make sure the files are readable by all.

## Extrinsics
Next, try following this tutorial: [Extrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/ExtrinsicCalibration). A required package, [camera_pose_calibration](http://wiki.ros.org/camera_pose_calibration) doesn't exist in Hydro. Clone it and build it from [jbohren's fork](https://github.com/jbohren-forks/camera_pose/tree/hydro-devel). Also check if the [official repo](https://github.com/ros-perception/camera_pose) has a Hydro version.

In the tutorial, it asks you to record `/camera/rgb/image_rect`, which doesn't exist for us. I think the correct topic is `camera/rgb/image_rect_mono`.

## PR2 calibration
The [PR2 calibration](http://wiki.ros.org/pr2_calibration) tutorial works pretty well. The only thing is that you should run `/etc/ros/distro/openni_head.launch` to start the Kinect.
