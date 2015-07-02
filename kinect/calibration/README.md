## Calibration

### Calibration and launch files
Find your kinect's serial number printed on the bottom of the kinect. Or, launch the kinect with `openni_launch` and the serial number will be printed to output. If there is a subdirectory here containing the serial number, no need to calibrate! Just transplant the files as follows:

* There are two intrinsic parameter files (e.g. `depth_B00366627745048B.yaml` and `rgb_B00366627745048B.yaml`)
  * Place them in a config directory in your ros package
* Extrinsic parameters are in the launch file, e.g. `kinect_nodes_B00366627745048B.launch`. To run in a calibrated mode, the kinect driver must always be launched with the launch file containing extrinsic parameters.
  * Copy your kinect's `kinect_node_[serial].launch` into the `launch` directory of your ros package
  * Replace the intrinsics URLs with the actual path of your intrinsics files.

### Steps
* To calibrate a new kinect, follow [these steps](https://github.com/hcrlab/wiki/blob/master/kinect/calibration/README.md)
* Check in the resulting files to this repo
