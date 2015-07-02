## Calibration

### Calibration and launch files
Find your kinect's serial number printed on the bottom of the kinect. Or, launch the kinect with `openni_launch` and the serial number will be printed to output. If there is a subdirectory here containing the serial number, no need to calibrate! Just transplant the files as follows:

* Extrinsic parameters are in the launch file, e.g. `kinect_nodes_B00366627745048B.launch`. To run in a calibrated mode, the kinect driver must always be launched with the launch file containing extrinsic parameters.
  * Copy your kinect's `kinect_node_[serial].launch` into the `launch` directory of a ros package
  * Replace the intrinsics URLs with the actual path of your intrinsics files.
* There are two intrinsic parameter files (e.g. `depth_B00366627745048B.yaml` and `rgb_B00366627745048B.yaml`)
  * Place them in your repo and include their path in your launch file (as in `kinect_node.launch`).

### Steps
* To calibrate a new kinect, follow [these steps](https://github.com/hcrlab/wiki/blob/master/kinect/calibration/README.md)
* Check in the resulting files to this repo
