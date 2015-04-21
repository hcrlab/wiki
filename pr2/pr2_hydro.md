# How to use the PR2 with Hydro

There are a couple of bugs with PR2 Hydro. Until the packages get updated, the following steps are necessary.

## Tucking the arms
In the Hydro version of pr2_tuck_arms_action, tuck_arms.py only starts the action server, but doesn't send a goal to tuck the arms.

For now, either write a SimpleActionClient that tucks the arms yourself, or get a different version of pr2_common_actions [(jstnhuang/pr2_common_actions)](https://github.com/jstnhuang/pr2_common_actions). If you have that, then you can run:
`rosrun pr2_tuck_arms_action tuck_arms_main.py -lt -rt`

## Teleop
PR2 teleop general fails because it can't find a .yaml file for pr2_mannequin mode. This has been fixed in the latest version of pr2_mannequin mode, but has not made its way into the Ubuntu repositories yet. The solution is to build pr2_apps with the latest code from Github.

```bash
cd ~/catkin_ws/src
git clone git@github.com:PR2/pr2_apps.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=hydro -y
catkin_make
source devel/setup.bash
```

## Laser scan
In Hydro, all the scripts in `pr2_mechanism_controllers` are not marked as executable for some reason. The way to fix this is to clone `pr2_mechanism_controllers` and mark the scripts as executable. Or, a hacky solution is to just edit the files in the system directory:

```
cd /opt/ros/hydro/lib/pr2_mechanism_controllers
sudo chmod +x *.py
```

Once you do this, you should now be able to start the tilt laser with:
```xml
<node pkg="pr2_mechanism_controllers" type="send_periodic_cmd_srv.py"
  name="laser_tilt_controller_3dnav_params" args="laser_tilt_controller linear 10 1.02 .31" />
```
