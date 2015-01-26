# Running Gazebo on a remote machine

Gazebo is great for simulating the robot. However, it requires a lot of computing power, and might not run well on your laptop. This is especially true if you're running inside a virtual machine.

One possible solution is to have one of the lab machines, which are very powerful, run Gazebo. Your computer would just communicate with Gazebo over the internet, leaving your computer free to do other things. This guide explains how to set this up, although the solution is not perfect.

We will assume that the host machine running Gazebo is named *labmachine*

# Host setup
The [recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md) has a `hostrobot` command. On the host machine, run:
```
hostrobot
roslaunch pr2_gazebo pr2_empty_world.launch
```

# Connecting to the remote Gazebo
We can take advantage of ROS's distributed nature, by using an existing command we already know.
```
setrobot labmachine
```


The hostrobot and setrobot commands make your computer and the lab computer agree on the URI for the ROS master. Setting the DISPLAY variable on the lab machine is also important, otherwise gazebo will crash. This allows Gazebo to actually open up a window on the lab machine. It may be possible to run Gazebo in a UI-less state, see this [ROS Q&A link](http://answers.ros.org/question/9324/run-gazebo-remotely/) for more information.

## Troubleshooting
If you aren't getting tf data, that may be an indication that the times on the computers are out of sync. One way to solve this problem is to just restart the lab machine. Another way to fix it is to sync the clocks on both machines, using the command `sudo ntpdate ntp.ubuntu.com`.

See the ROS wiki on [network setup](http://wiki.ros.org/ROS/NetworkSetup#Timing_issues.2C_TF_complaining_about_extrapolation_into_the_future.3F) for details.
