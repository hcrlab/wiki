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
Add c1 as 10.68.0.1:
```
10.68.0.1  c1
```

- Restart the laptop. Verify that its IP address on eth0 is 10.68.0.7 using ifconfig.

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
export ROS_HOSTNAME=hal9k
```
