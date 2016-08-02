# Lab demos
There are a variety of demos you can run in the lab.
- [Teleoperation RWS app](#demo-teleoperation-app)
- [PbD actions RWS app](#demo-pbd-actions-app)
- [CodeIt! RWS app](#demo-codeit-app)
- [Programming by Demonstration](#demo-programming-by-demonstration)
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

#### Demo: Teleoperation app
This will allow you to move the head and base, as well as tuck the arms.
You should tuck the arms before moving the base.

Known issues:
- Occasionally, the video can become extremely slow and delayed

#### Demo: PbD actions app
You can execute an action that was previously programmed using Programming by Demonstration.

Notes:
- You should wait about 10 seconds after visiting the app to run actions
- You can often tell when PbD has initialized because the grippers will close and the head will look ahead
- PbD actions tend to be slow to run the first time, but speed up afterwards.
  This may be due to caching.

#### Demo: CodeIt! app
You can create and run programs with CodeIt!
You can create your own small programs, or run an app like "Load laundry"

Notes:
- If you want to run a PbD action inside a program, you must open the PbD actions app first
- You should wait about 10 seconds after visiting the app to run programs
- The "Load laundry" app works best if the basket is not too full of clothes.
  Run it a few times before the demo to make sure it is operating as well as possible.
  When the robot stuffs clothes into the "laundry machine", the grippers should just barely graze the outside.
- The error "Failed to reset program screen on stop" is normal, it just means that you don't have Blinky open
- If the system is working, you should see the run button turn into a red stop button when a program is run.
  Occasionally this can be delayed, but it's always a good sign to see it.
- Check the log files using `rqt_pr2_dashboard` to look for potential problems.
- More log files can be found using:
  ```
  cd /var/ros/logs/`rosparam get run_id`
  tail -f js_interpreter-1-stdout.log # Use tab completion to find the right file
  tail -f pr2_pbd_interaction-5.log # Use tab completion to find the right file
  ```

#### Troubleshooting
##### RWS fails to start up
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

##### An app is not working as expected
You can close and relaunch an app

1. Click "Home" and refresh the page
1. All the apps that are currently running will have "X" buttons next to them
1. Close the app that isn't working by clicking the X.
   Note that there's no feedback to clicking the X.
1. Wait for 10 seconds for the app to fully close on the robot.
1. Refresh the page and visit the app again
1. Most apps have some kind of startup time, so it's a good idea to wait another 10 seconds after visiting an app page to start using it.

### Demo: Programming by Demonstration
To actually do programming by demonstration, you need to launch the frontend separately.

1. To start the backend, go to RWS and click on "PbD actions"
1. Plug in the wireless microphone to your computer.
   Check that the sound input is not too quiet or loud.
1. On that computer, make sure you are running the latest version of [pr2_pbd](https://github.com/PR2/pr2_pbd) on Hydro.
   Check the [README](https://github.com/PR2/pr2_pbd/blob/hydro-devel/README.md) or the [.travis.yml](https://github.com/PR2/pr2_pbd/blob/hydro-devel/.travis.yml) to see how to install all the dependencies.
1. Run the frontend using `roslaunch pr2_pbd_interaction pbd_frontend.launch`

- Also see the instructions for [running PbD without RWS](https://github.com/hcrlab/wiki/blob/master/pbd/README.md)

## Beam
1. Ask for access to the Beam
1. If you are on Windows or Mac, download the Beam client from the [Suitable website](https://suitabletech.com/installers).
1. If you are on Linux, you can install the [Beam Chrome extension](https://chrome.google.com/webstore/detail/beam/onglbhicnlbbljbhkilnnkbokcgoheej?hl=en-US)
1. The software requires a working camera/microphone setup to operate the Beam
