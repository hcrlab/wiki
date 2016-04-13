# Recommended .bashrc
We like to add the following to our .bashrc.

There are a few things to customize:

- `source /opt/ros/hydro/setup.bash`: Change this to default to groovy, hydro, etc.
- `source ~/catkin_ws/devel/setup.bash`: Point this to the correct path for your catkin workspace.
- `MY_IP=$(/sbin/ifconfig eth0 ...`: `eth0` is the name of the network device. If you are on a laptop, you most likely will need to change `eth0` to `wlan0`. You can check the list of network devices by typing `ifconfig`.
- The terminal prompt `PS1` can be customized. As given, it will turn your terminal prompt purple, and show the current hostname, the current path, and the current ROS Master host.
- The ROSCONSOLE_FORMAT [can be modified to your liking](http://wiki.ros.org/rosconsole#Console_Output_Formatting). The recommended version prints the node, function, and line number where the logging occurred (if it can be determined).

```bash
source /opt/ros/hydro/setup.bash # Default to hydro. Change this to default to another distro if you want.
source ~/catkin_ws/devel/setup.bash # Change this to point to your catkin_ws.
export ROS_HOSTNAME=localhost # Optional, the name of this computer.
export ROS_MASTER_HOST=localhost # Used to inform us what robot we're connected to.
export ROS_MASTER_URI=http://localhost:11311 # The location of the ROS master.
export ROBOT=sim # The type of robot.
export ROSCONSOLE_FORMAT='${node} ${function}:${line}: ${message}' # Formats log messages, see http://wiki.ros.org/rosconsole#Console_Output_Formatting

# Get IP address on ethernet
# If you're on a desktop, change wlan0 to eth0
function my_ip() {
    MY_IP=$(/sbin/ifconfig wlan0 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
    echo ${MY_IP:-"Not connected"}
}

# Terminal prompt formatting, optional.
# Makes your terminal look like [host (c1) ~/dir], in purple.
# Search for "bash ps1" online to learn more.
PS1='\[\e[1;35m\][\h \w ($ROS_MASTER_HOST)]$ \[\e[m\]'

# Run "setrobot sim" to go to simulation.
# Run "setrobot c1" to connect to Rosie.
# Run "setrobot softshell" to connect to a turtlebot.
# Note "setrobot c1" is equivalent to the realrobot command that is on most machines.
function setrobot() {
  if [ "$1" = "sim" ]; then
    export ROS_HOSTNAME=localhost;
    export ROS_MASTER_HOST=localhost;
    export ROS_MASTER_URI=http://localhost:11311;
    export ROBOT=sim;
  else
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1;
    export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
    export ROS_IP=`my_ip`;
  fi
}

# Turn your computer into a Gazebo server that others can use.
# You run: hostrobot, then roslaunch pr2_gazebo...
# Then others can run: setrobot labmachinename
function hostrobot() {
  export ROS_HOSTNAME=labmachinename.cs.washington.edu
  export ROS_MASTER_URI=http://labmachinename.cs.washington.edu:11311
  export DISPLAY=:0
  export ROS_IP=`my_ip`
  export ROBOT=sim
}

# Run this from the root of your catkin_ws to run rosdep update.
function getdeps() {
  rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
}

# Advanced, used to switch between groovy and hydro.
# Assumes you have ~/catkin_ws_groovy and ~/catkin_ws_hydro
# Call "setros groovy" or "setros hydro" to switch between them.
# Runs setup commands for ROS stuff.
function setros() {
  source /opt/ros/$1/setup.bash
  source ~/catkin_ws_$1/devel/setup.bash --extend
}
```

## Other resources
Check out Mike Chung's [dotfiles](https://github.com/mjyc/dotfiles)
