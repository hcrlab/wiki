# How to calibrate the Kinect

## Intrinsics
Calibrate the intrinsics using this tutorial: [Intrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). Save the files to `/etc/ros/camera_info`, and make sure the files are readable by all.

Later, you can load the intrinsic calibration parameters by launching openni with:
```
<include file="$(find openni_launch)/launch/openni.launch">
  <arg name="rgb_camera_info_url"   default="file:///etc/ros/camera_info/rgb_A00362A08919053A.yaml" />
  <arg name="depth_camera_info_url" default="file:///etc/ros/camera_info/depth_A00362A08919053A.yaml" />
</include>
```

## Extrinsics
Next, try following this tutorial: [Extrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/ExtrinsicCalibration). However, we need to make several changes.

### Set up camera_pose_calibration
A required package, [camera_pose_calibration](http://wiki.ros.org/camera_pose_calibration) doesn't exist in Hydro. Clone it and build it from [jbohren's fork](https://github.com/jbohren-forks/camera_pose/tree/hydro-devel). Also check if the [official repo](https://github.com/ros-perception/camera_pose) has a Hydro version.

### Set up the augmented IR
The calibration package tries to find the checkerboard in the IR image. You should cover the IR projector with scotch tape or a post-it note to diffuse the speckle pattern. However, this causes the image to be too dark to find a checkerboard in. The solution is to use the contrast package. Clone it, build it, and run:

```
rosrun contrast contrast_augmenter image:=/camera/ir/image_rect_ir
```

The calibration package expects a `/camera/ir/image_rect` topic to exist by default. On our robot, that doesn't exist for some reason, but we have `/camera/ir/image_rect_ir`. Running the node above causes the contrast node to republish a higher-contrast version of `/camera/ir/image_rect_ir` to `camera/ir_augmented/image_raw`.

### Record data
In the tutorial, it asks you to record `/camera/rgb/image_rect`, which doesn't exist for us. I think the correct topic is `camera/rgb/image_rect_mono`:

```rosbag record --limit=300 -O rgb.bag /camera/rgb/image_rect /camera/rgb/camera_info```

### Follow the tutorial
Follow the tutorial, stopping when it says "Start the calibrator."

### Hack the calibrator
For some reason, the calibrator always fails when displaying the visualization, with the error: `Assertion failed (src.type() == dst.type()) in cvResize`. The only thing that really worked for me was to comment that line out altogether. Luckily, it is just part of the visualization, so the calibration window won't show any data, but the calibration will still run.

The line is the first `cv.Resize` call in `render(self, window)` in `capture_monitor.py` in `camera_pose_calibration`.

### Start the calibrator
We will start the calibrator, but adjust the arguments so that we use the high-contrast IR image, and use the right checkerboard size (assuming we're using the small checkerboard that comes with the PR2 accessory kit).
```
roslaunch camera_pose_calibration calibrate_2_camera.launch camera1_ns:=/camera/rgb_bag camera2_ns:=/camera/ir_augmented camera2_image:=/image_raw checker_rows:=4 checker_cols:=5 checker_size:=0.0245
```

### Play the bag again
Again, we follow the tutorial, but adjust an argument to use `image_rect_mono` instead of just `image_rect`.
```
rosbag play rgb.bag --clock /camera/rgb/camera_info:=/camera/rgb_bag/camera_info /camera/rgb/image_rect_mono:=/camera/rgb_bag/image_rect
```

The calibration visualization should show random white noise for both images, but if everything's set up properly, the checkerboard should be detected in both images. You should see the grids overlaid over the white noise, and they should be in approximately the same place.

The data is saved to the cache file referenced earlier in the ROS tutorial.

## PR2 calibration
The [PR2 calibration](http://wiki.ros.org/pr2_calibration) tutorial works pretty well. The only thing is that you should run `/etc/ros/distro/openni_head.launch` to start the Kinect.
