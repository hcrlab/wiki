# Logging in ROS and where to find log files

- [roscpp logging overview](http://wiki.ros.org/roscpp/Overview/Logging)
- [rospy logging overview](http://wiki.ros.org/rospy/Overview/Logging)
- [rosconsole docs](http://wiki.ros.org/rosconsole)

## Changing output format
This is a helpful line to add to your .bashrc:
```bash
export ROSCONSOLE_FORMAT='${node} ${function}:${line}: ${message}'
```

For every logging message you print, it will print out the node that emitted the message.
It will also show the function name and line number if debugging information is available.
You can modify this to your liking, see [rosconsole's output formatting docs](http://wiki.ros.org/rosconsole#Console_Output_Formatting)

## Where to find logs
First, find out the current `run_id`: `rosparam get run_id`.
The logs are written to `~/.ros/logs/{RUN_ID}`
For RWS, the logs are written to `/var/ros/logs/{RUN_ID}`
