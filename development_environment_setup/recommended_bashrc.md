We like to add the following to our .bashrc.

There are a few things to customize:

- `source /opt/ros/groovy/setup.bash`: Change this to default to groovy, hydro, etc.
- `source ~/catkin_ws/devel/setup.bash`: Not included in this .bashrc. Add it if you frequently work with a particular catkin workspace.
- `MY_IP=$(/sbin/ifconfig eth0 ...`: `eth0` is the name of the network device. If you are on a laptop, you most likely will need to change `eth0` to `wlan0`. You can check the list of network devices by typing `ifconfig`.

```bash
# Call "setros groovy" or "setros hydro" to switch between them.
# Runs setup commands for ROS stuff.
function setros() {
  source /opt/ros/$1/setup.bash
}
source /opt/ros/groovy/setup.bash # Default to groovy. Change this to default to Hydro if you want.
export ROS_HOSTNAME=localhost # Optional, the name of this computer.
export ROS_MASTER_URI=http://localhost:11311 # The location of the ROS master.
export ROBOT=sim # The type of robot.

# Get IP address on ethernet
# If you're on a laptop, change eth0 to wlan0
function my_ip() {
    MY_IP=$(/sbin/ifconfig eth0 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
    echo ${MY_IP:-"Not connected"}
}

# Run "setrobot sim" to go to simulation.
# Run "setrobot c1" to connect to Rosie.
# Run "setrobot softshell" to connect to a turtlebot.
# Note "setrobot c1" is equivalent to the realrobot command that is on most machines.
function setrobot() {
  if [ "$1" = "sim" ]; then
    export ROS_HOSTNAME=localhost;
    export ROS_MASTER_URI=http://localhost:11311;
    export ROBOT=sim;
  else
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
    export ROS_IP=`my_ip`;
  fi
}
```
