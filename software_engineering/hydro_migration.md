# Migrating Packages from Groovy to Hydro

The official [ROS guide to hydro migration](http://wiki.ros.org/hydro/Migration) is a good resource. It's also pretty long though.

The main things to consider are:

- Package functionality have changed between Groovy and Hydro (i.e. functions that you used to call no longer exist, message definitions have changed).
- Packages might not exist anymore.

The first two cases have similar solutions. If package functionality is missing, try to check if that functionality has moved elsewhere. Or see if you can adapt your code to use the new functionality. A lot of times when things like message definitions change, it is easy to slightly adapt your code to use the new message definition.

- Packages may have become catkinized, meaning that maybe you can catkinize your package that was relying on some previously rosbuild packages (see the [catkin migration guide](http://wiki.ros.org/catkin/migrating_from_rosbuild) for help)

Note: One thing to note when catkinizing, if you have a Python package remember to include a setup.py file if necessary and uncomment the catkin_python_setup() line in your CMakelists.txt. Rosbuild would automatically include the src folder of a package in the PYTHONPATH, but catkin doesn't necessarily do that. Details are in the [catkin migration guide](http://wiki.ros.org/catkin/migrating_from_rosbuild#Python).




