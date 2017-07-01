Generally, `freenect_launch` has worked better for us when using the Kinect.
However, if you must use OpenNI (such as when using someone else's code that uses OpenNI), here is how to get the Kinect working.

Instructions are for ROS Indigo on Ubuntu 14.04.

# Test if OpenNI works
A simple way to test if OpenNI already works is to run:
```
roslaunch openni_launch openni.launch
```

If you do not already have this installed, run:
```
sudo apt-get install ros-indigo-openni-launch
```

# Install Kinect driver
If you get an error message, "No devices found...", then you probably need to install a Kinect driver.
See instructions [here](http://learn.turtlebot.com/2015/02/01/5/), replicated below:
```
mkdir ~/kinectdriver 
cd ~/kinectdriver 
git clone https://github.com/avin2/SensorKinect 
cd SensorKinect/Bin/
tar xvjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
```
