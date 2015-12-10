## Switching Between Groovy And Hydro
This wiki describes how to switch between ROS distributions, including some of
what happens under the hood. 

### TLDR;
```
> ssh c1
> robot claim
> robot groovy
> source /etc/ros/distro/setup.bash
> robot start
> export ROS_PACKAGE_PATH=~/my_groovy_workspace/:$ROS_PACKAGE_PATH
```

### Preliminaries
You will need to ssh onto the robot and claim it before you can continue.
```
> ssh c1
> robot claim
```

### Setting the ROS environment to groovy (Before robot start)
In the interest of giving a concrete example, I'm going to describe how to switch to groovy.
If you want to switch to hydro, substitute "hydro" for "groovy" in all the commands below.
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
The directory `/etc/ros/groovy` is a separate thing from `/opt/ros/groovy`. The `robot` command and the directory `/etc/ros/groovy` only exist on the robot. If you have multiple ROS distributions installed on your desktop, you can switch to groovy using `source /opt/ros/groovy/setup.bash` instead of the commands above.


### Start the robot
Now that ROS is set to the right distribution, you can start the robot. If the robot was already started
under a different distribution, you need to start it again.
```
> robot start
```

### Set `ROS_PACKAGE_PATH` (in each console window)

Now you have to set `ROS_PACKAGE_PATH` to the right thing. `ROS_PACKAGE_PATH` determines where ROS looks for packages
whose executables you may try to run with `rosrun` or `roslaunch`. Sourcing `/etc/ros/distro/setup.bash` (as we did above)
clears your `ROS_PACKAGE_PATH` and resets it to the minimum required for the current distribution. 

If you want access to packages in one of your workspaces, you need to add it to `ROS_PACKAGE_PATH`.
Since `ROS_PACKAGE_PATH` is an environment variable, you will have to do this in every console window where you 
plan to use packages from the workspace:
```
> export ROS_PACKAGE_PATH=~/my_groovy_workspace/:$ROS_PACKAGE_PATH
```
This command concatenates your workspace to `ROS_PACKAGE_PATH`. If your workspace is built with catkin 
(as are most hydro workspaces), there is a shortcut to do this (`source ~/my_hydro_workspace/devel/setup.bash`). 
If your workspace is built with rosbuild, you must do it manually.

### Run your stuff!
Now you can run your code.

