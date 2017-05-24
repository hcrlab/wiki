# Lab demos
There are a variety of demos you can run in the lab.
- [Teleoperation RWS app](#demo-teleoperation-app)
- [Code3](#demo-code3)
- [Fetch PbD](#fetch-pbd)
- [Fetch CodeIt](#fetch-codeit)
- [Beam](#beam)

## PR2
### RWS
RWS is the web interface that runs various applications for the robot.
With the exception of CodeIt!, most apps will display well on small devices like phones and tablets.

Common directions to start RWS:

1. Go to RWS and log in with your Google account
1. If you get the "Robot is starting up" message, wait approximately 30 seconds for the robot to start up before opening other apps.
   Do not reload the page for at least a minute, as this may cause the robot to try starting up twice.
1. You should hear the fans start to whir after about 30 seconds.
1. Go to the Dashboard page and check that the runstop is on and that the motors are enabled

### Demo: Teleoperation app
This will allow you to move the head and base, as well as tuck the arms.
You should tuck the arms before moving the base.

Known issues:
- Occasionally, the video can become extremely slow and delayed

### Demo: Code3
Code3 is our mobile manipulation programming system.

Code3 is preinstalled on the robot.
To run it, follow these steps in order:
1. Make sure RWS is running.
1. (Recommended) Run the PR2 dashboard: `rosrun rqt_pr2_dashboard rqt_pr2_dashboard`
1. (Recommended) Run the pr2_pbd frontend: `roslaunch pr2_pbd_interaction pbd_frontend`
1. In RWS, first open the PbD actions app. Wait for "Interaction initialized" to appear in the dashboard.
1. In RWS, open CodeIt! in a new tab. Wait for "CodeIt! for the PR2 is ready" in the dashboard.
1. Take one of the Nexus 7 tablets and go the the PR2's Blinky webpage.
1. Select a program in CodeIt! and run it. If the "Run" button doesn't turn into a "Stop" button, refresh the page.

A typical demo is to run the tic-tac-toe program or the program that gets/replaces the shoulder-mounted tool.

If you want to install/run Code3 on your own computer, refer to these resources:
- [Code3 installation instructions](https://github.com/hcrlab/code3/wiki/Installation)
- [Code3 User Guide](https://github.com/hcrlab/code3/wiki/User-guide)

Notes:
- You should wait about 10 seconds after visiting the app to run actions
- You can often tell when PbD has initialized because the grippers will close and the head will look ahead
- Check the log files using `rqt_pr2_dashboard` to look for potential problems.
- More log files can be found using:
  ```
  cd /var/ros/logs/`rosparam get run_id`
  tail -f js_interpreter-1-stdout.log # Use tab completion to find the right file
  tail -f pr2_pbd_interaction-5.log # Use tab completion to find the right file
  ```

### Troubleshooting for RWS apps on the PR2
#### RWS fails to start up
If you get the "Robot is starting up" message when you go to RWS, wait for 30 seconds to a minute and listen for fans inside the robot to start whirring.
If you do not hear this whirring after a while, the startup may have failed.

1. Check the number of running processes by running `robot users`
1. A successful startup will have over 80 processes running, whereas a failed startup may be stuck at around 10 processes
1. If the number of processes is stuck at a low number, stop everything with `robot stop -f`
1. Verify that the processes are stopped with `robot users`.
   A few RWS processes may still be running, which is normal (it's the Apache web server)
1. Visit the RWS homepage and wait again for the whirring sound
1. If this doesn't work, repeat this process while watching the Apache error log: `tail -f /var/ros/rws_error.log`.
1. The most common issue is "run_id did not match run_id on the parameter server."
   Try shutting everything down, and restarting the Apache web server with `sudo apache2ctl stop` and then `sudo apache2ctl start`.

#### An app is not working as expected
You can close and relaunch an app

1. Click "Home" and refresh the page
1. All the apps that are currently running will have "X" buttons next to them
1. Close the app that isn't working by clicking the X.
   Note that there's no feedback to clicking the X.
1. Wait for 10 seconds for the app to fully close on the robot.
1. Refresh the page and visit the app again
1. Most apps have some kind of startup time, so it's a good idea to wait another 10 seconds after visiting an app page to start using it.

## Fetch
### Fetch PbD
There is a [Fetch Docs page](http://docs.fetchrobotics.com/fetch_pbd.html) for the general demo. This details some of the features of the demo.  
Specific instructions for our lab:  

1. SSH into astro using the hcrlab account.  
   Optional to make sure no one else's stuff is running:  
   1. `sudo service robot stop`  
   1. `sudo service robot start`  
1. `source ~/pbd_demo_ws/devel/setup.bash`
1. `roslaunch fetch_pbd_interaction pbd.launch`
1. Go to astro:8080 or (astro.cs.washington.edu:8080) in your browser (also works on mobile) for the frontend interface.

Check out the docs page linked above for fancy commandline options when launching the backend. 

Also if there are any weird bugs, you can clear the couchdb as a last resort, use this command on the robot:
`curl -X DELETE http://127.0.0.1:5984/fetch_pbd`

### Fetch CodeIt
You can run Fetch CodeIt by visiting `astro/` or `astro.cs.washington.edu` in a web browser and clicking on CodeIt!.
So far the only functionality is to open/close the gripper.

## Beam
1. Ask for access to the Beam
1. If you are on Windows or Mac, download the Beam client from the [Suitable website](https://suitabletech.com/installers).
1. If you are on Linux, you can install the [Beam Chrome extension](https://chrome.google.com/webstore/detail/beam/onglbhicnlbbljbhkilnnkbokcgoheej?hl=en-US)
1. The software requires a working camera/microphone setup to operate the Beam
