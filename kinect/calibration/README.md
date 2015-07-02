## Calibration

### Calibration and launch files
Find your kinect's serial number printed on the bottom of the kinect. Or, launch the kinect with `openni_launch` and the serial number will be printed to output. If there is a subdirectory here containing the serial number, no need to calibrate!

* There are two ways to use intrinsic parameter files (e.g. `depth_B00366627745048B.yaml` and `rgb_B00366627745048B.yaml`)
  1. Place them in `~/.ros/camera_info/`. That is the default place that the driver looks for them when you run `roslaunch openni_launch openni.launch`.
  2. Place them in your repo and include their path in your launch file (as in `kinect_node.launch`)

* Extrinsic parameters should go in a launch file. To run in a calibrated mode, the kinect driver must always be launched with the launch file containing extrinsic parameters.
* You will need to edit the launch file to use it. Replace the intrinsics URLs with the actual path of your intrinsics files, or if using option 1 for intrinsics, delete the intrinsics URL arg nodes.

