# Skeleton tracking with openni_tracker

[openni_tracker](http://wiki.ros.org/openni_tracker) allows you to track a person's skeleton using a Kinect. It also gives you the positions, relative to the camera frame, of the person's head, torso, hands, knees, elbows, etc.

In this tutorial, we assume that we have a separate Kinect, mounted on a tripod somewhere in the room, connected to a separate laptop. However, most of these instructions are the same in case you want to try people tracking with a Kinect mounted on a robot.

## Attempt 1: Installation and startup
Install openni_tracker:
```sudo apt-get install ros-hydro-openni-tracker```

Run `roscore`.

Launch openni_tracker:
```rosrun openni_tracker openni_tracker```

You should get this error:
```Find user generator failed: This operation is invalid!```

## Attempt 2: Making openni_tracker work
After searching a bit online, there are a couple of resources that come up. According to [How to get the Kinect skeleton tracking working on ROS](http://www.therobotstudio.com/software/how-to-get-the-kinect-skeleton-tracking-working-on-ros/), everyone has this same error. The problem has something to do with us not having the right OpenNI libraries. Unfortunately, the OpenNI project also shut down recently, so they don't have a website anymore. Fortunately, there are some sites that mirror the downloads we want.

Download [nite-bin-linux-x64-v1.5.2.21.tar.bz2](http://www.mira-project.org/downloads/3rdparty/bin-linux/nite-bin-linux-x64-v1.5.2.21.tar.bz2) from [http://www.mira-project.org/downloads/3rdparty/bin-linux/](http://www.mira-project.org/downloads/3rdparty/bin-linux/). This is assuming you are using a 64-bit lab computer, otherwise, go to the page and download the x86 version.

Unpack and install it:
```
tar xf nite-bin-linux-x64-v1.5.2.21.tar.bz2
sudo ./uninstall.sh
sudo ./install.sh
```

Now try openni_tracker:
```rosrun openni_tracker openni_tracker```

Stand in front of the Kinect. If the terminal window where you ran openni_tracker, you should see output like this:
```
[ INFO]: New User 1
[ INFO]: Calibration started for user 1
[ INFO]: Calibration complete, start tracking user 
```

If you see "calibration failed", make sure that you are getting point cloud data.

## Visualizing the data
What is openni_tracker supposed to do? According to the [documentation](http://wiki.ros.org/openni_tracker), the positions of the person's head, torso, arms, legs, etc. are supposed to be published as tf transforms. That is, each body part will be considered its own coordinate frame, and openni_tracker publishes the transformation necessary to convert a body part coordinate frame to the camera's coordinate frame.

Now open up rviz: `rosrun rviz rviz`

As usual, the fixed frame is "map", which doesn't exist. The fixed frame can be changed in the "Displays" panel on the left, under "Global Options". Which fixed frame should we change it to? Well, the only one that doesn't look like a body part is "openni_depth_frame", so that's what we'll go with.

Now add a tf display. You should see a bunch of wiggling lines and arrows. To make it easier to visualize, uncheck "Show arrows" and set the scale to 0.5.

You should see frames representing your head, arms, legs, etc. Try moving around and seeing if it looks right to you.

## Overlaying the data with a point cloud
So far, we just see a bunch of coordinate axes floating in space. Just to be sure that the skeleton tracking is correct, we might want to overlay the tf data with an actual point cloud.

To get point cloud data, we will launch openni_launch: `roslaunch openni_launch openni.launch`

Try adding a PointCloud2 display. The status section should turn red with an error. If you open it up and click on the transform error, it will say:
```For frame [/camera_rgb_optical_frame]: Frame [/camera_rgb_optical_frame] does not exist```

This is because we set our fixed frame to openni_depth_frame previously. Right now, openni_tracker has the position of the skeleton relative to openni_depth_frame, and openni_launch has the point cloud relative to camera_link (or a variety of equivalent coordinate frames), but they don't actually know that openni_depth_frame and camera_link are actually the same.

In rviz, try switching the fixed frame between /camera_link and /openni_depth_frame. Notice that if you set the fixed frame to /camera_link, then the point cloud shows up, but if you change it to /openni_depth_frame, you can only see the skeleton.

Luckily, we can get both. If you at the parameters section of the [openni_tracker documentation](http://wiki.ros.org/openni_tracker#Parameters), you'll see that you can specify the camera frame ID for skeleton tracking by setting the camera_frame_id parameter. Let's change the camera frame ID to /camera_link.

First, type `rosnode list` and note that the openni_tracker's node's name is just `/openni_tracker`. We'll use this later.

Now kill openni_tracker.

To change this, we need to use the ROS parameter server. If you look at the source code for openni_tracker, you'll see that [it uses a private namespace](https://github.com/ros-drivers/openni_tracker/blob/hydro-devel/src/openni_tracker.cpp#L185) for its node handle. You can read all about [ROS naming](http://wiki.ros.org/Names).

The consequence of this is that all parameters need to be prefixed with the name of the node (/openni_tracker in this case) instead of saying:
~~```rosparam set camera_frame_id camera_link```~~

Enter:
```rosparam set /openni_tracker/camera_frame_id camera_link```

Now run openni_tracker again. You should see the skeleton overlaid with the point cloud data.
