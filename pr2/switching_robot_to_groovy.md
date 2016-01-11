# Switching the Robot to Groovy
This wiki describes how to switch between ROS distributions, including some of
what happens under the hood. 

In the interest of giving a concrete example, I'm going to describe how to switch to groovy.
If you want to switch to hydro, substitute "hydro" for "groovy" in all the commands below.

## TLDR;
```
> ssh c1
> robot claim
> robot groovy
> source /etc/ros/distro/setup.bash
> source ~/my_groovy_workspace/devel/setup.bash --extend
> robot start
```

## Preliminaries
You will need to ssh onto the robot and claim it before you can continue.
```
> ssh c1
> robot claim
```

## Setting the ROS environment to groovy (Before robot start)

```
> robot groovy
```
The command above sets a simlink in the filesystem. We can see it with:
```
hcrlab@c1:~/catkin_ws_hydro$ ls -lah /etc/ros/distro
lrwxrwxrwx 1 root root 14 Dec  9 18:54 /etc/ros/distro -> /etc/ros/groovy
```
To have access to the ros commands in your chosen distribution, you will also have to source that distribution's `setup.bash`. You can do this with:
```
> source /etc/ros/distro/setup.bash
```
In this case, the command above is equivalent to 
```
> source /etc/ros/groovy/setup.bash
```
since we just created a simlink in the filesystem linking `/etc/ros/distro/` to `/etc/ros/groovy/`.

### Side Note (relevant to desktops running ROS)
The directory `/etc/ros/groovy` is a separate thing from `/opt/ros/groovy`. The `robot` command and the directory `/etc/ros/groovy` only exist on the robot. If you have multiple ROS distributions installed on your desktop, you can switch to groovy using `source /opt/ros/groovy/setup.bash` instead of the commands above (depending on your `.bashrc` file, you may use the `setros groovy' command instead, see [Recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md)).

## Overlay your groovy workspace (in each console window)
So far, you have set environment variables necessary to run the ROS packages that have been installed on the robot or on your computer.
Now, you need to set the environment variables for your workspace, where you have been developing code.

The most important of these, for running code, is `ROS_PACKAGE_PATH`. `ROS_PACKAGE_PATH` determines where ROS looks for packages
whose executables you may try to run with `rosrun` or `roslaunch`. Sourcing `/etc/ros/distro/setup.bash` (as we did above)
clears your `ROS_PACKAGE_PATH` and resets it to the minimum required for the current distribution. `setup.bash` also sets various environment variables for finding libraries when catkin builds code with CMake.

If you want access to packages in one of your workspaces, you need to overlay your workspace onto the current environment.
Since overlaying workspaces only changes environment variables, you will have to do this in every console window where you 
plan to use or build packages from the workspace:
```
> source ~/my_groovy_workspace/devel/setup.bash --extend
```
This command concatenates your workspace to `ROS_PACKAGE_PATH`.
Using `--extend` ensures the workspace is overlayed properly.

If your workspace is built with rosbuild, you must set `ROS_PACKAGE_PATH` manually:
```
> export ROS_PACKAGE_PATH=~/my_groovy_workspace/:$ROS_PACKAGE_PATH
```

### Convenience
You may find it tedious to enter one of the two commands above in every terminal window you create.
You can reduce the amount of typing necessary by creating a bash function and adding it to your .bashrc:
```
function myworkspace() {
  source /opt/ros/groovy/setup.bash
  source ~/my_groovy_workspace/devel/setup.bash --extend
}
```

Then, in each terminal window, you can just type:
```
> myworkspace
```

## Start the robot
Now that ROS is set to the right distribution, you can start the robot. If the robot was already started
under a different distribution, you need to start it again.
```
> robot start
```

## Debugging
Depending on your `.bashrc`, some of the commands here, especially the ones meant to modify your `ROS_PACKAGE_PATH` (like `source /etc/ros/distro/setup.bash` and `export ROS_PACKAGE_PATH=~/my_groovy_workspace/:$ROS_PACKAGE_PATH`) will not be necessary. If the behavior is different than expected, check your `.bashrc` and try echoing your `ROS_PACKAGE_PATH` after every step to make sure it's what you expect. 

## Run your stuff!
Now you can run your code.

