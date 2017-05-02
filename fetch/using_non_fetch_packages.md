# Using Non-Fetch ROS Packages

**Note: Packages that are not from the Fetch-approved repos are guaranteed to work** 

Use these instructions if you want a certain package from the general ROS repositories (usually because the Fetch repositories have an old version of the package and the ROS repos have a new version).
To specify that you want specific packages from the ROS repositories you need to "pin" those packages.

### Do this only if ```/etc/apt/sources.list.d/ros-original.list``` does not exist
There should already be a file in ```/etc/apt/sources.list.d``` called ```ros-original.list``` that includes the general ROS repositories.
If this file does not exist, you can make one:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-original.list'
sudo apt-get update
```
This basically just adds the regular ROS repositories to your sources. 

### Do this only if ```/etc/apt/preferences.d/ros-original-pin-400``` does not exist
In  ```/etc/apt/preferences.d``` there should be a file called ```ros-original-pin-400```. 
This file tells apt to make packages from the regular ROS repositories lower priority (400 vs. the regular priority of 500).
This means that if a package exists in the Fetch repositories, that package will be used instead of the one from the ROS repositories. 
If this file does not exist, you can add it. The contents of the file are:
```
Package:  *
Pin: origin packages.ros.org
Pin-Priority: 400
```

## Pinning Your Packages!
This is probably the only step you will have to take. Create a file called something like ```your-package-name-pin-500```.
For example, to use the ar_track_alvar package from the ROS repositories we made ```ar-track-alvar-pin-500```.
Here's an example of the contents of the file:
```
Package:  ros-indigo-ar-track-alvar ros-indigo-ar-track-alvar-msgs 
Pin: origin packages.ros.org 
Pin-Priority: 500
```
You will change the first line to reflect the names of the packages you want to include.
Notice we include multiple packages that will be needed. 

After creating this file, run the following to confirm that your pinning worked:
```bash
sudo apt-get update
apt-cache policy
```
The output of that last command should show you that your packages are now listed under "Pinned Packages" at the end of the output. 
Now if you do ```sudo apt-get install your-package``` it should get installed from the proper source.

This information was from [this AskUbuntu post](https://askubuntu.com/questions/170235/how-do-i-cherry-pick-packages-from-a-ppa) and [this Ubuntu documentation](https://help.ubuntu.com/community/PinningHowto).
