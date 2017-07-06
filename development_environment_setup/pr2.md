# Getting set up on the PR2

The PR2 has two computers on it: c1 and c2. c1 is the main computer that deals with the filesystem, networking, and running ROS, while c2 handles most of the sensor data. c2 is set to replicate c1's filesystem and maybe some other things. You can think of the robot as a computer just like any other.

## Create an account

Have someone in the lab ssh into c1 and create you an account. The account creation can be done with
```bash
sudo adduser <uname>
```

They need to make you an admin as well; most guides online say that you need to belong to the group 'sudo', but empirically it looks like 'admin' is the one that currently works on Rosie.
```bash
sudo adduser <uname> admin
sudo usermod -a -G rosadmin <uname>
```

Now, have them logout and then try ssh'ing into Rosie2 using $ ssh <uname>@c1. After this, test to see that you have admin rights by doing
```bash
sudo ls
```

## Env loader
Add the following line to the end of your .bashrc on the robot. This only needs to be done on the PR2.
```bash
export ROS_ENV_LOADER=/etc/ros/distro/env.sh
export ROS_HOSTNAME=mayarobot-wired
export ROS_MASTER_HOST=mayarobot-wired
export ROS_MASTER_URI=http://mayarobot-wired:11311
export ROBOT=pr2
export KINECT1=true
```

Note /etc/ros/distro is a symlink to either /etc/ros/indigo.

You should also edit your .bashrc according to our [recommended .bashrc](recommended_bashrc.md).
The recommended .bashrc will tell you to set ROS_MASTER_URI, etc. to different values.
On the robot, the values listed above should take precedence.

## First login
The first time you log in, the system will ask you if you want to set up SSH keys.
You should say yes.
This refers to setting up SSH keys between the two computers of the robot, which is highly recommended.
You can read our [SSH guide](ssh.md) to learn more about other SSH-related tasks you can do.
