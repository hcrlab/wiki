# Programming by Demonstration (PbD)

## How to run the demo

This section describes how to run the RViz-based PbD system for a lab demo. It assumes that the computer used for the demo has already been configured properly (see below if you would like to set it up on a new computer or different account).

**Summer 2015:** Please use Justin's desktop computer, with the 'hcrlab' account.

#### Making sure you're on groovy
As of 12/2015, pbd only works under groovy. Use this [tutorial](https://github.com/hcrlab/wiki/blob/master/pr2/switching_between_groovy_and_hydro.md) to understand how to switch between groovy and hydro.

#### Starting the demo

* Log in to the hcrlab account.
* Make sure the microphone is connected to the desktop.
* Terminal 1: Start the robot
```
ssh hcrlab@c1
robot claim
robot start
```
* Terminal 2: Run the dashboard
```
setrobot c1
rosrun rqt_pr2_dashboard rqt_pr2_dashboard
```
* Activate motors from the kill switch (press green button)
   * Make sure the robot's arms are stiff
   * Make sure the dashboard is clear (everything is green)
* Back in Terminal 1: Start the back-end of PbD
```
roslaunch pr2_pbd_interaction pbd_backend.launch
```
* Terminal 3: Start the front-end of PbD
```
setrobot c1
roslaunch pr2_pbd_interaction pbd_frontend.launch
```
* Turn on the headset of the microphone and test by saying "Test microphone"; the robot should repond with a chime and nod.

#### Programming the robot

List of commands available [here](commands.pdf) as a PDF (printed copy will be available at the lab).

**Basic commands:** You can demonstrate different arm stiffnesses, moving the arms around, and changing the hand states with the following commands:
```
OPEN/CLOSE RIGHT/LEFT HAND
FREZE RIGHT/LEFT HAND
RELAX RIGHT/LEFT HAND
```

**Simple actions** (no objects or dummy objects): Wave
* Make sure the robot's right arm is relaxed and its right hand is open. Create an empty action using command:
```
CREATE NEW ACTION
```
* Move the arm to consecutive waving poses and add them as poses into the action using command:
```
SAVE POSE
```
* Make the robot replay the action using command:
```
EXECUTE ACTION
```

**Complex actions** (with objects): Push object
* Place a table in front of the robot and place the ellipse red plate on the table.
* Create another action and then make the robot detect the plate:
```
CREATE NEW ACTION
RECORD OBJECT POSE
```
* Check RViz to make sure the green bounding box of the detected object is aligned with the plate
* Save four poses to push the object:
   * Pose 1: Starting pose away from the object, to the side of the robot (absolute)
   * Pose 2: Pre-push pose almost touching the object from the direction of Pose 1 (relative)
   * Pose 3: The pose that the robot reaches after slightly pushing the object (relative)
   * Pose 4: Retract pose similar to Pose 1
* Execute the action with different poses of the plate

Complex actions can involve picking up objects and plading them relative to other objects. Note that a pose is automatically saved when you open/close the robot's hand.

### Troubleshooting

* Cannot ssh to the robot
   * Network problems
* The robot responds to GUI commands but not speech commands:
   * The microphone is off (light on the headset is off) -- turn it on using the small switch (ligth should turn green).
   * The microphone is out of battery (light on the headset is red) -- change the two AA bateries of the headset
   * Pocketsphinx did not start or crashed -- check Terminal 3 for errors.
   * Microphone volume is not adjusted properly. Go to System Settings > Sounds > Input, make sure the microphone input volume is around 100% (not above). Say commands and check Terminal 3 to see if the commands are recognized properly.
* The robot does not become stiff
    * The kill switch is out of battery
* RViz does not show anything
   * You forgot to do ```setrobot c1``` in Terminal 3
* PbD GUI does not show up
* The robot does not see objects
* ...


## Getting set up
- The PbD demo currently works in ROS groovy. Follow the [Development environment setup](https://github.com/hcrlab/wiki/tree/master/development_environment_setup) before installing it.
- Installation instructions are available at [PR2/pbd Github README](https://github.com/PR2/pr2_pbd#pr2-programming-by-demonstration)
- Older but simpler version of the installation instructions are also availabe in this [Google doc](https://docs.google.com/document/d/1N7hqa6YVgZ_CNCbshKKQrnkInWHVPQBgjMxb6hwm3mI/edit?usp=sharing) (also includes instructions for setting up the microphone)
