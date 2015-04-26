# Installing ROS

If you are going to work with the PR2, we recommend installing ROS Groovy on Ubuntu 12.04.

If you are working with the Turtlebot, you can install a newer version of ROS, such as Hydro.

## 1. Install ROS
Run the following scripts to install ROS.

**install_ros_groovy.sh**: Installs ROS Groovy. See the [official instructions](http://wiki.ros.org/groovy/Installation/Ubuntu) for more information.

**install_ros_hydro.sh**: Installs ROS Hydro. See the [official instructions](http://wiki.ros.org/groovy/Installation/Ubuntu) for more information.

**install_utils.sh**: Installs miscellaneous utilities (vim, git, meld, and tmux).

**install_pr2_groovy.sh**: Installs PR2 packages (for Groovy). Run `rosdep update` after running this script. See the [official instructions](http://wiki.ros.org/Robots/PR2/groovy) for more information.

**install_pr2_hydro.sh**: Installs PR2 packages (for Hydro). Run `rosdep update` after running this script. See the [official instructions](http://wiki.ros.org/Robots/PR2/hydro) for more information.

**install_turtlebot.sh**: Installs Turtlebot packages (for Hydro). See the [official instructions](http://wiki.ros.org/turtlebot/Tutorials/hydro/Installation) for more information.

**install_hydro_all.sh**: Installs utils, ROS Hydro, and PR2 packages for Hydro.

Installation should take about an hour.

Example:
```
sudo ./install_utils.sh
sudo ./install_ros_groovy.sh
rosdep update
sudo ./install_pr2_groovy.sh
```
