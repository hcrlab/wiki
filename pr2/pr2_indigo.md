# Using the PR2 in Indigo

There are a couple of things to be mindful of when transitioning to Indigo on the PR2.

## Connecting to the robot
- The new name of the robot is `mayarobot-wired`.
- Make sure that the robot is plugged into the building network via the robot's WAN port.
- SSH using `ssh mayarobot-wired`
- Connect via ROS using by typing `export ROS_MASTER_URI=http://mayarobot-wired:11311` into every terminal window.
- The updated `setrobot` command from the [recommended .bashrc](../development_environment_setup/recommended_bashrc.md) has been updated with a special case to handle this, so you can type `setrobot c1` as usual.

We do not recommend unplugging the robot and using the wireless network.
Because its batteries are no longer useful, the robot has to be plugged in for power, anyway.
However, if you *need* the robot to use the wireless network:
- You can still access the robot through a laptop by connecting to the rosie2LAN wireless network.
- SSH using the command `ssh 10.68.0.1`
- On your laptop, you will want to edit `/etc/hosts` and add a mapping from `c1` to `10.68.0.1`, and then restart networking with `sudo service networking restart`. Be sure to undo this once you're done, so that you can resolve the `c1` name after disconnecting from rosie2LAN.
- On the robot, type `sudo route del default gw 128.208.5.100 dev wan0`. This is necessary for the robot to access the internet, it routes the robot's internet through its wireless router instead of through the WAN port.
- When you are done, plug the network cable back into robot's WAN port and type (on the robot) `sudo route add default gw 128.208.5.100 dev wan0`. This ensures that the robot will use the wired connection instead of the wireless connection.

## Setup / migration
### Install Ubuntu 14.04
ROS Indigo is only supported on Ubuntu 13.10 and 14.04.
We recommend installing 14.04, which is a long term release.

### Install ROS Indigo and PR2 Indigo packages
- [ROS Indigo installation](http://wiki.ros.org/indigo/Installation/Ubuntu)
- When needed, you can install PR2 Indigo packages by searching for `ros-indigo-pr2-*`

### Update your .bashrc
See [recommended .bashrc](../development_environment_setup/recommended_bashrc.md).
Change it so that instead of sourcing `/opt/ros/hydro/setup.bash` it sources `/opt/ros/indigo/setup.bash`

- Add `export KINECT1=true` to your .bashrc on the robot to make the Kinect work.
- Also add it to your desktop's .bashrc for Gazebo simulations.

### Migrate your code
#### catkin workspace
Create a new workspace for Indigo packages.
We recommend you use the new [catkin tools](https://catkin-tools.readthedocs.io/en/latest/index.html).
```bash
# Make sure you have updated your .bashrc, then open a new terminal window.
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws_indigo/src
cd ~/catkin_ws_indigo
catkin init
catkin build
```

#### CMakeLists.txt
If you use Eigen, numpy, or  in your code, then you will need to add a `<build_depend>cmake_modules</build_depend>` to your CMakeLists.txt.

If you depend on OpenCV, then make your package depend on `cv_bridge`.

- See the [official ROS Indigo migration guide](http://wiki.ros.org/indigo/Migration)
- Also see the [catkin Indigo migration guide](http://docs.ros.org/indigo/api/catkin/html/adv_user_guide/catkin_migration_indigo.html)
