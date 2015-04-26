# Running ROS on a VirtualBox Virtual Machine

## Running RViz in Ubuntu 12.04, OSX 10.10 host

These instructions assume you have installed VirtualBox on your host machine, and have created an Ubuntu VM (also known as a guest machine).

In order to run RViz, I did a few things:

1. Upgrade OpenGL on the guest machine. Run `sudo apt-get install libgl1-mesa-dri`.
2. Use software rendering. Run `export LIBGL_ALWAYS_SOFTWARE=1`. Optionally add this to your `~/.bashrc` to avoid running it every time.

Now you should be able to run `rosrun rviz rviz`.

If your host machine has a good graphics card, you may be able to improve performance by skipping step 2 above and instead trying these steps:

2. Install the VirtualBox guest extensions. To do this, while running your VM, go to Device -> Insert Guest Additions CD Image.
3. Enable 3D graphics acceleration for the VM. To do this, shut down your VM, select it from the list of VMs, and go to Settings -> Di
splay -> Video -> Extended Features, clicking "Enable 3D Acceleration".
