# Backpack configuration / using the PR2 outside the lab
These instructions tell you how to use the PR2 with a computer that's plugged into the service port. This is useful for conferences or demos outside the CSE building, where there's no CSE network or wifi. This computer could also be attached to the PR2 itself as a backpack. The rest of the instructions assume the latter case but keep in mind that the same steps apply  in either scenario.

## Laptop configuration
- Plug the laptop into the robot's service port.

- Make sure /etc/hostname is just rosie-backpack

- Edit /etc/network/interfaces

We are using 10.68.0.7 as the static address of the backpack laptop.
```
auto eth0
iface eth0 inet static
  address 10.68.0.7
  netmask 255.255.255.0
```

- Edit /etc/hosts
Add c1 as 10.68.0.1 and c2 as 10.68.0.2:
```
10.68.0.1  c1
10.68.0.2  c2
```

- Restart networking with `sudo restart networking`. Verify that its IP address on eth0 is 10.68.0.7 using ifconfig. If that doesn't work, then try to restart networking in some other way, or simply restart the laptop.

## Robot configuration
- Edit /etc/hosts and add the backpack:
```
10.68.0.7  rosie-backpack
```

## Verify
You should be able to ssh from c1 to rosie-backpack and vice-versa.

## ROS configuration
On the laptop, these environment variables need to be set.
```
export ROS_MASTER_URI=http://c1:11311
export ROS_HOSTNAME=rosie-backpack
```

If you are using the `setrobot` command from the [recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md), modify it or create another bash function such that the `ROS_MASTER_URI` is set using `export ROS_MASTER_URI=http://$1:11311;`
