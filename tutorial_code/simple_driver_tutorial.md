#  Simple Driver Tutorial

This tutorial will take you through the step-by-step process of driving the PR2 for the first time. If you have any questions. Ask somsone in the lab.

# Make a Catkin Workspace
If you have already made a catkin workspace, skip this step and just make sure you have that workspace sourced.
Otherwise, follow [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to make yourself a catkin workspace on your machine. Be careful, to make a ROS Hydro workspace. The tutorial (at the time of my writing it) uses ROS Indigo, so just replace "indigo" with "hydro". Source the `devel/setup.bash` from your workspace.

# Make a Package
Inside your workspace, you can now make a catkin package. 
- Navigate to the `src` folder of your catkin workspace. 
- Run `catkin_create_pkg pr2_drive_example std_msgs geometry_msgs rospy`. This creates a package called `pr2_drive_example` with dependencies on  `std_msgs`, `geometry_msgs` and `rospy`.
- Navigate into your new package. 
- Make a scripts directory for the node we're about to write: `mkdir scripts`. Generally we put the executable files into the `scripts` directory. The `src` directory is for libraries.

# Driving the PR2
Just in case we accidentally drive the robot into something valuable on our first try, we're going to use the simulator. 
- Open up a new terminal.
- Make sure your ROS distribution is set to Hydro. 
- Run: `roslaunch pr2_gazebo pr2_empty_world.launch`. This will launch the Gazebo simulator with the PR2 robot.

Let's figure out how to drive the robot. 
- New terminal again, also set to Hydro
- Type: `rostopic list`. This gives a list of all the available topics on the PR2. One of them is the topic that listens to driving commands. There are way too many to really tell which is the driving topic. Luckily I'll tell you.
- The topic we want is called `/base_controller/command`. 
- Do: `rostopic info /base_controller/command`. This tells us information about the topic. Notice that the topic type is `geometry_msgs/Twist`. That's the type of message the topic takes. 
- Do: `rosmsg show geometry_msgs/Twist`. This shows the information in a Twist message. It makes sense that a driving topic would listen to messages that have linear and angular velocities.
- You can actually manually publish to that topic right now. Run: `rostopic pub /base_controller/command geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'`. That command publishes a Twist message telling the robot to drive 0.5m/s forward. In Gazebo, you should see the robot move forward slightly. It might be hard to notice, so try running the command a couple of times. 

# Writing the Node
So now we know how to make the robot move. Let's do that with Python code now. Go back to your original terminal, which is hopefully still in your package. This example is pretty trivial. We're going to write a node that listens on a certain topic for a speed and then makes the robot drive forward at that speed. 
 - Navigate into your scripts folder.
 - Make a file `driver.py`.
 - All python executables start with: 
 ```python
 #!/usr/bin/env python
 ```
 - Then we want to import the right ROS packages. We'll need `rospy` for sure. We'll also need the Twist message from `geometry_msgs` in order to command the robot to drive. Let's also include the Float64 message from `std_msgs`, because it will listen for a speed in the form of a float:
```python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
```
- We'll make a Driver class for our node. The Driver class will subscribe to the `/drive_forward` topic to listen for those speeds that we'll send it. It will publish to `/base_controller/command` in order to drive the robot.
```python
class Driver:
    def __init__(self):
        rospy.Subscriber("/drive_forward", Float64, self.forward_callback)
        self.cmdPub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
```
- Let's write the callback for the subscriber. This defines what to do when it receives a message on the `/drive_forward` topic. We'll take the float we receive and use it as the linear velocity in the x direction. The robot's positive x axis points forward.
```python
    def forward_callback(self, data):
        twist_msg = Twist()
        twist_msg.linear.x = data.data
        self.cmdPub.publish(twist_msg)
```
- Now we have our class, we just need to make a main function. We basically want to make an instance of our class and then we want that instance to stay alive so that it's subscriber can receive messages and execute its callback. We first initialise the node. Then we create and instance of our class. Then we use `rospy.spin()`, which prevents Python from exiting while your node is still alive. 
```python
if __name__ == '__main__':
    rospy.init_node('driver_node')
    Driver()
    rospy.spin()
```
- As a last step, make sure your node is executable: `chmod +x driver.py`.
Full code for this tutorial can be found here. 

# Build Your Node
Technically your node is in Python so nothing really needs to get built, but go to the root of your catkin workspace and catkin_make it anyway.

# Run the Node
- Type `rosrun pr2_drive_example driver.py`. Unless there are terrible errors, your node is ready.
- In a new terminal, with your ROS distribution set to Hydro. 
- Type: `rostopic list | grep drive_forward`. Hopefully your new topic shows up.
- Type: `rostopic pub /drive_forward std_msgs/Float64 0.5`. This tells the robot to drive forward at 0.5m/s. You should see the robot move in Gazebo. Again, it might be subtle, so try running it a few times.

# Extensions
Try running this on the real robot now, if it works. Or try expanding the functionality to drive the robot in different directions and turn. Maybe instead of listening to a topic, the node could take keyboard input. 


