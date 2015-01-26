# How to set up your environment.

## 1. Installing Ubuntu
If you do not already have Ubuntu, we recommend installing Ubuntu 12.04 on your computer. We're currently not using newer versions of Ubuntu because the PR2 uses Groovy or Hydro, which are not supported on the latest versions of Ubuntu. See [REP 3 -- Target Platforms](http://www.ros.org/reps/rep-0003.html) for more information.

## 2. Installing ROS
Run the install scripts given in [installing_ros](https://github.com/hcrlab/wiki/tree/master/development_environment_setup/installing_ros).

Example:
```
sudo ./install_utils.sh
sudo ./install_ros_hydro.sh
rosdep update
sudo ./install_pr2_hydro.sh
```

## 3. Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

## 4. Set up your .bashrc
Copy the [recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md) to the end of your ~/.bashrc file. Be sure to update it according to what ROS distro you're using, whether you're on a desktop or laptop, etc.
