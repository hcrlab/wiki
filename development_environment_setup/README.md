# How to set up your environment.

## 1. Installing Ubuntu
If you do not already have Ubuntu, we recommend installing Ubuntu 12.04 on your computer. We're currently not using newer versions of Ubuntu because the PR2 uses Groovy or Hydro, which are not supported on the latest versions of Ubuntu. See [REP 3 -- Target Platforms](http://www.ros.org/reps/rep-0003.html) for more information.

## 2. Installing ROS
Run the install scripts given in [installing_ros](https://github.com/hcrlab/wiki/tree/master/development_environment_setup/installing_ros).

Example:
```
sudo ./install_utils.sh
sudo ./install_ros_indigo.sh
rosdep update
sudo ./install_pr2_indigo.sh
```

## 3. Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

## 4. Set up your .bashrc
Copy the [recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md) to the end of your ~/.bashrc file. Be sure to update it according to what ROS distro you're using, whether you're on a desktop or laptop, etc.

## 5. Set up SSH
See the [SSH guide](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/ssh.md). It describes many things you can do with SSH, but you will at least want to:

1. Create SSH keys
2. Add your public key to Github

## PR2
If you are creating an account on the PR2, see the guide for [getting set up on the PR2](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/pr2.md).

## If you don't have Ubuntu
[Running Gazebo on a remote machine](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/remote_gazebo.md). Gazebo running slowly on your laptop? See how to run Gazebo on a lab computer instead.

[Running ROS in a virtual machine](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/virtual_machine.md). Specifically, setting up graphics so that rviz runs smoothly.

[Setting up with a Mac](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/mac_setup.md). Note that using ROS on a Mac is still not well supported.

## Coding tools
[Automatic code formatting](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/auto_code_formatting.md). Automatically format your code to match style guides and to spend less time dealing with whitespace.

[Vim tips](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/vim.md). Setting up vim for ROS development.

[QT creator](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/qt_creator.md). Setting up QT Creator as an IDE.

[tmux](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/tmux.md). A terminal multiplexer that allows you to create multiple windows inside of a single SSH session. Also, if your SSH session dies, the tmux session will stay exactly the way it was, and you can "reattach" to it.

## Other tips
[Make your development environment pretty!](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/cosmetics.md)

[Mike's dotfiles](https://github.com/mjyc/dotfiles). What are [dotfiles](http://dotfiles.github.io/)? In fact, it is shameless adaptation (copy) from [Paul Irish's](https://www.youtube.com/user/paulirish/videos) [dotfiles](https://github.com/paulirish/dotfiles).
