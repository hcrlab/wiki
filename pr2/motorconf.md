# Motorconf
You may need to use the motorconf tool to rename a device.
In particular, you may need to rename a caster that you are swapping into the robot (e.g., taking the front left caster of one robot and placing it into the back left of your robot).

- [Motorconf documentation](http://wiki.ros.org/ethercat_hardware#motorconf)

To do this on the robot, follow these steps:

## Get motorconf working
1. `cd /opt/ros/indigo/lib/ethercat_hardware`
1. `sudo su`
1. `./motorconf`
1. If you get "error loading shared libraries: libeml.so," run `export LD_LIBRARY_PATH=/opt/ros/indigo/lib` and try again.
1. `./motorconf`

## Figure out what needs to be renamed
Ideally, save the output of `./motorconf -i ecat0 -a /opt/ros/indigo/share/ethercat_hardware/actuators.conf` before swapping in the new caster, so you can compare with output after swapping.
A caster consists of three motors: `_caster_r_wheel_motor`, `_caster_l_wheel_motor`, and `_caster_rotation_motor`, in that order.
Figure out which 3 devices (in the case of a caster) need to be renamed, and which device numbers they are.

The caster motors are typically listed in this order:

Device # | Name
-------- | ----
2        | br_caster_r_wheel_motor
3        | br_caster_l_wheel_motor
4        | br_caster_rotation_motor
5        | fl_caster_r_wheel_motor
6        | fl_caster_l_wheel_motor
7        | fl_caster_rotation_motor
8        | bl_caster_r_wheel_motor
9        | bl_caster_l_wheel_motor
10       | bl_caster_rotation_motor
11       | fr_caster_r_wheel_motor
12       | fr_caster_l_wheel_motor
13       | fr_caster_rotation_motor

## Rename them
Let's say you've installed a "front left" caster into the back left position.
Devices 8-10 would have the `fl` prefix instead of the `bl` prefix.
Rename them to have the `bl` prefix like so:
```
./motorconf -i ecat0 -a /opt/ros/indigo/share/ethercat_hardware/actuators.conf -n bl_caster_r_wheel_motor -d 8 -p
./motorconf -i ecat0 -a /opt/ros/indigo/share/ethercat_hardware/actuators.conf -n bl_caster_l_wheel_motor -d 9 -p
./motorconf -i ecat0 -a /opt/ros/indigo/share/ethercat_hardware/actuators.conf -n bl_caster_rotation_motor -d 10 -p
```

## Test
Run startup and the motor calibration and then try teleoping the robot around.
It should drive as expected.
