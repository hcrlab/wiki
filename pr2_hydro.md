# How to use the PR2 with Hydro

There are a couple of bugs with PR2 Hydro. Until the packages get updated, the following steps are necessary.

## Tucking the arms
In the Hydro version of pr2_tuck_arms_action, tuck_arms.py only starts the action server, but doesn't send a goal to tuck the arms.

For now, either write a SimpleActionClient that tucks the arms yourself, or get a different version of pr2_common_actions [(jstnhuang/pr2_common_actions)](https://github.com/jstnhuang/pr2_common_actions).

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
