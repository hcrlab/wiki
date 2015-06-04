# Troubleshooting Gazebo

## Robot falling through the floor in Gazebo
If this happens to you when you create a new account (or out of the blue), here are the two things to do:
Make sure you have the latest version of Gazebo. You can download it with instructions given here (go with the debs option): http://gazebosim.org/#download
Run Gazebo without ROS to make it download the latest robot models. Do this by opening a terminal window and typing gazebo; when the simulator window opens (allow ~10 seconds), it will have an empty world; go to the 'Insert' tab on the left panel, make sure the models are accumulated in the dropdown menu, and try adding a PR2 into the world.
After this, the exact same command that made PR2 fall off the sky should now work fine.

## Unmet Dependencies When Installing/Uninstalling Gazebo
If you have unmet dependency issues that can't be solved by "apt-get -f install", try removing ROS sources from your sources list. Then do "apt-get update" and then try to remove and install the Gazebo packages again.
