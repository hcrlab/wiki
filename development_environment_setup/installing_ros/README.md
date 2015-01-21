# Installing ROS

If you are going to work with the PR2, we recommend installing ROS Groovy on Ubuntu 12.04.

If you are working with the Turtlebot, you can install a newer version of ROS, such as Hydro.

Run the following scripts to install ROS.

**install_ros_groovy.sh**: Installs ROS Groovy. See the [official instructions](http://wiki.ros.org/groovy/Installation/Ubuntu) for more information.

**install_ros_hydro.sh**: Installs ROS Hydro. See the [official instructions](http://wiki.ros.org/groovy/Installation/Ubuntu) for more information.

**install_utils.sh**: Installs miscellaneous utilities (vim, git, meld, and tmux).

**install_pr2.sh**: Installs PR2 packages (for Groovy). See the [official instructions](http://wiki.ros.org/Robots/PR2/groovy) for more information.

**install_turtlebot**: Installs Turtlebot packages (for Hydro). See the [official instructions](http://wiki.ros.org/turtlebot/Tutorials/hydro/Installation) for more information.

## Sample
```
sudo ./install_utils.sh
sudo ./install_ros_groovy.sh # or sudo ./install_ros_hydro.sh
sudo ./install_pr2.sh # or sudo ./install_turtle.sh
```
