# How to calibrate the Kinect

## Intrinsics
Calibrate the intrinsics using this tutorial: [Intrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). Save the files to `/etc/ros/camera_info`, and make sure the files are readable by all.

Later, you can load the intrinsic calibration parameters by launching openni with:
```xml
<include file="$(find openni_launch)/launch/openni.launch">
  <arg name="rgb_camera_info_url"   default="file:///etc/ros/camera_info/rgb_A00362A08919053A.yaml" />
  <arg name="depth_camera_info_url" default="file:///etc/ros/camera_info/depth_A00362A08919053A.yaml" />
</include>
```

## Extrinsics
Next, try following this tutorial: [Extrinsic calibration](http://wiki.ros.org/openni_launch/Tutorials/ExtrinsicCalibration). However, we need to make several changes.

### Resources
- [Arnaud Ramey](https://sites.google.com/site/rameyarnaud/research/ros/kinect-calibration)
- [ROS tutorial](http://wiki.ros.org/openni_launch/Tutorials/ExtrinsicCalibration)

### Set up camera_pose_calibration
A required package, [camera_pose_calibration](http://wiki.ros.org/camera_pose_calibration) doesn't exist in Hydro. Clone it and build it from [jbohren's fork](https://github.com/jbohren-forks/camera_pose/tree/hydro-devel). Also check if the [official repo](https://github.com/ros-perception/camera_pose) has a Hydro version.

### Set up the augmented IR
The calibration package tries to find the checkerboard in the IR image. You should cover the IR projector with scotch tape or a post-it note to diffuse the speckle pattern. However, this causes the image to be too dark to find a checkerboard in. The solution is to use the [contrast](https://github.com/hcrlab/contrast/tree/master) package. Clone it, build it, and run:

```
rosrun contrast contrast_augmenter image:=/camera/ir/image_rect_ir
```

The calibration package expects a `/camera/ir/image_rect` topic to exist by default. On our robot, that doesn't exist for some reason, but we have `/camera/ir/image_rect_ir`. Running the node above causes the contrast node to republish a higher-contrast version of `/camera/ir/image_rect_ir` to `camera/ir_augmented/image_raw`. You man not see any visible change in the image feed.

Another issue is that `image_view` does not always decode IR images correctly. If you can't verify the IR image using `image_view` because it is too dark, instead verify it in Rviz.

1. `rosrun rviz rviz`
2. Click "Add" button, select "Image" (not "Camera", which needs transforms to work)
3. Set "Image Topic" to `camera/ir_agumented/image_raw`

### Record data
In the tutorial, it asks you to record `/camera/rgb/image_rect`, which doesn't exist for us. The correct topic is `camera/rgb/image_rect_mono`:

```rosbag record --limit=300 -O rgb.bag /camera/rgb/image_rect_mono /camera/rgb/camera_info```

### Follow the tutorial
Follow the tutorial, stopping when it says "Start the calibrator." Then come back and finish the calibration by following the steps below.

### Hack the calibrator
For some reason, the calibrator always fails when displaying the visualization, with the error: `Assertion failed (src.type() == dst.type()) in cvResize`. The only thing that really worked for me was to comment that line out altogether. Luckily, it is just part of the visualization, so the calibration window won't show any data, but the calibration will still run.

The line is the first `cv.Resize` call in `render(self, window)` in `capture_monitor.py` in `camera_pose_calibration`.

### Start the calibrator
We will start the calibrator, but adjust the arguments so that we use the high-contrast IR image, and use the right checkerboard size (assuming we're using the small checkerboard that comes with the PR2 accessory kit).
```
roslaunch camera_pose_calibration calibrate_2_camera.launch camera1_ns:=/camera/rgb_bag camera2_ns:=/camera/ir_augmented camera2_image:=image_raw checker_rows:=4 checker_cols:=5 checker_size:=0.0245
```

### Play the bag again
Again, we follow the tutorial, but adjust an argument so that the recorded topic `image_rect_mono` plays back as `image_rect`, since the calibrator listens for `image_rect` by default.
```
rosbag play rgb.bag --clock /camera/rgb/camera_info:=/camera/rgb_bag/camera_info /camera/rgb/image_rect_mono:=/camera/rgb_bag/image_rect
```

The calibration visualization should show random white noise for both images, but if everything's set up properly, the checkerboard should be detected in both images. You should see the grids overlaid over the white noise, and they should be in approximately the same place.

The data is saved to the cache file referenced earlier in the ROS tutorial.

### Get the calibration parameters
The calibrator writes calibration parameters to a single message under the topic `camera_calibration` in the cache file you specified in the launch file for the extrinsic transform publisher. The simplest way to read the message stored in the bag is by writing a script that uses the [Python rosbag API](http://wiki.ros.org/rosbag/Code%20API).

The message should contain two `camera_pose` messages representing the poses of the rgb and depth cameras. The first will be close to the identity transformation. The second will have somewhat larger values.


### Appying the calibration

You can apply the calibration when you launch the kinect by suppressing the transforms that the kinect normally publishes and replacing them with your own static transforms. Create a new launch file along the lines of `kinect_node.launch` file from [Arnaud Ramey's tutorial](https://sites.google.com/site/rameyarnaud/research/ros/kinect-calibration) (scroll all the bottom). To run the kinect in a calibrated mode, you will always have to launch it using the new launch file.

This snippet from `kinect_node.launch` publishes static transform between `head_mount_kinect_rgb_optical_frame` and `head_mount_kinect_depth_optical_frame` using the second transformation from the cache file:

```xml
<node pkg="tf" type="static_transform_publisher" name="$(arg camera)_extrinsic_calibration"
    args="-0.058386075423 -0.0420297233163 -0.0047490001086
      -0.0293246565925 0.0227688658924 -0.000634651560892 0.999310382453
      $(arg tf_prefix)/$(arg camera)_rgb_optical_frame $(arg tf_prefix)/$(arg camera)_depth_optical_frame
      100" />
```

### Cheat sheet
This is a recap of all the commands to run for the extrinsic calibration, assuming you've read all the directions above. You will need 5 terminal windows.

#### Setup
```
T1: roscore
T5: rosrun contrast contrast_augmenter image:=/camera/ir/image_rect_ir
```

#### Run the kinect, and verify that it's running
```
T3: rosparam set /use_sim_time false
T2: roslaunch pa_perception extrinsic_calibration.launch  # or the name of the launch file you created earlier
- check image feed in rviz: /camera/ir/image_raw
- check image feed in rviz: /camera/ir_augmented/image_raw
- check image feed in rviz: /camera/rgb/image_color
- check image feed in rviz: /camera/rgb/image_rect_mono
- turn off image feed in rviz
```

#### Record RGB bag
```
T3: rosparam set /use_sim_time false  # this line intentionally repeated
T3: rosbag record --limit=300 -O rgb.bag /camera/rgb/image_rect_mono /camera/rgb/camera_info
T3: ^C  # after waiting 10 seconds
T3: rosbag info rgb.bag
- make sure there are both `camera_info` and `image_rect_mono` messages
```

#### Restart the kinect and initialize it with sim time
```
T3: rosparam set /use_sim_time true
T2: ^C
T2: roslaunch pa_perception extrinsic_calibration.launch  # or the name of the launch file you created earlier
T3: rosbag play rgb.bag --clock  # don't skip this step
```

#### Calibrate using live IR and recorded RGB
```
T4: roslaunch camera_pose_calibration calibrate_2_camera.launch camera1_ns:=/camera/rgb_bag camera2_ns:=/camera/ir_augmented camera2_image:=image_raw checker_rows:=5 checker_cols:=4 checker_size:=0.0245
T3: rosbag play rgb.bag --clock /camera/rgb/camera_info:=/camera/rgb_bag/camera_info /camera/rgb/image_rect_mono:=/camera/rgb_bag/image_rect
```

## PR2 calibration
The [PR2 calibration](http://wiki.ros.org/pr2_calibration) tutorial works pretty well. The only thing is that you should run `/etc/ros/distro/openni_head.launch` to start the Kinect.
