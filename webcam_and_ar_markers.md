# How to use the webcam with AR markers

This guide is a bit specific to when you want to use AR markers with a laptop.

We will use [gscam](http://wiki.ros.org/gscam) to get the webcam image, and [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) to get the AR tracking.

## Install the webcam
```sudo apt-get install ros-hydro-gscam```

## Run the webcam
```roslaunch gscam v4l.launch```

Launch rviz, and add an Image display. You should see the webcam image.

## Install the AR tracker
```sudo apt-get install ros-hydro-ar-track-alvar```

## Set up the AR tracker launch files
The AR tracker package only has PR2-specific launch files. We will copy the PR2, image-based launch file into our own package.

```
cd ~/catkin_ws/src
catkin_create_pkg webcam_ar_track
mkdir webcam_ar_track/launch
cd webcam_ar_track/launch
cp /opt/ros/hydro/share/ar_track_alvar/launch/pr2_indiv_no_kinect.launch webcam_indiv.launch
```

Open webcam_indiv.launch, and change the following arguments to point to the webcam:

```
<arg name="cam_image_topic" default="/v4l/camera/image_raw" />
<arg name="cam_info_topic" default="/v4l/camera/camera_info" />
<arg name="output_frame" default="/v4l_frame" />
```

Now run catkin_make, and run the node:
```
roslaunch webcam_ar_track webcam_indiv.launch
```

Inspect the output of the AR tracker:
```
rostopic echo /ar_pose_marker
```

You should see the ID of the marker show up. Note that we don't have the position or orientation of the marker, just the ID.
