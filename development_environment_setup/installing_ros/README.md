# Installing ROS

If you are going to work with the PR2, we recommend installing ROS Indigo on Ubuntu 14.04.

## 1. Install ROS
Run the following scripts to install ROS.

**install_ros_indigo.sh**: Installs ROS Indigo. See the [official instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) for more information.

**install_utils.sh**: Installs miscellaneous utilities (vim, git, meld, and tmux).

**install_pr2_indigo.sh**: Installs PR2 packages (for Indigo). Run `rosdep update` after running this script. See the [official instructions](http://wiki.ros.org/Robots/PR2/indigo) for more information.

**install_turtlebot.sh**: Installs Turtlebot packages (for Hydro). See the [official instructions](http://wiki.ros.org/turtlebot/Tutorials/hydro/Installation) for more information.

Installation should take about an hour.

Example:
```
sudo ./install_utils.sh
sudo ./install_ros_indigoy.sh
rosdep update
sudo ./install_pr2_indigo.sh
```
