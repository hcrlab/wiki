# Using the PR2 in Indigo

There are a couple of things to be mindful of when transitioning to Indigo on the PR2.

## Install Ubuntu 14.04
ROS Indigo is only supported on Ubuntu 13.10 and 14.04.
We recommend installing 14.04, which is a long term release.

## Install ROS Indigo and PR2 Indigo packages
- [ROS Indigo installation](http://wiki.ros.org/indigo/Installation/Ubuntu)
- When needed, you can install PR2 Indigo packages by searching for `ros-indigo-pr2-*`

## Update your .bashrc
See [recommended .bashrc](../development_environment_setup/recommended_bashrc.md).
Change it so that instead of sourcing `/opt/ros/hydro/setup.bash` it sources `/opt/ros/indigo/setup.bash`

## Migrate your code
### catkin workspace
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

### CMakeLists.txt
If you use Eigen, numpy, or  in your code, then you will need to add a `<build_depend>cmake_modules</build_depend>` to your CMakeLists.txt.

If you depend on OpenCV, then make your package depend on `cv_bridge`.

- See the [official ROS Indigo migration guide](http://wiki.ros.org/indigo/Migration)
- Also see the [catkin Indigo migration guide](http://docs.ros.org/indigo/api/catkin/html/adv_user_guide/catkin_migration_indigo.html)

### Dashboard
Because `diagnostics_msgs` changed in Indigo, you can't use the PR2 Hydro dashboard with the 
