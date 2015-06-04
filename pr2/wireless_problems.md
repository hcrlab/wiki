# Troubleshooting PR2 wireless problems

## Getting into the PR2's internal network
- Plug into the service port OR
- Connect to the wireless access point, rosie2LAN

## Checking the router
Once on the network, you can visit the configuration page for the robot's router by going to 10.68.0.5. Verify that the settings are set up according to [Configure the robot wifi](https://pr2s.clearpathrobotics.com/wiki/PR2%20Manual/Chapter14#Configure_the_Robot_Wifi).

You can test if the robot has wireless internet by unplugging it, and routing a particular IP through the robot's router, as described in "Configure the robot wifi". If you can ssh into the robot, you can try to ping outside websites from the robot.

## Factory reset of the router
If you're not sure why the wireless router can't connect to the internet, you might just want to do a factory reset of the router. At that point, the router will show up as dd-wrt. You should connect to it and find the configuration page at 192.168.1.1. The username is root, and the password is admin. Perform the following steps:
- Under Setup->Basic setup->Network setup->Router IP, set the Local IP Address to 10.68.0.5
- Under Setup->Basic setup->Wireless Setup->WAN Connection Type, disable DHCP
- Under Wireless, disable one of the wireless physical interfaces, say wl0.
- For the other interface (wl1), set the wireless mode to Client and the network name to CSE-Local.

The "Configure the robot wifi" page explains how to connect the router to an unprotected wireless network that requires authentication later, such as school networks like CSE-Local. If the wireless network requires a password, then go to Wireless->Wireless security and set the security mode for wl0 or wl1 to the same security being used to protect the wireless network. Put the password in under WPA Shared Key.

## Useful resources
- [PR2 manual chapter 12: networking](https://pr2s.clearpathrobotics.com/wiki/PR2%20Manual/Chapter12#Networking)
- [PR2 manual chapter 13: computer configuration](https://pr2s.clearpathrobotics.com/wiki/PR2%20Manual/Chapter13#Computer_Configuration)
- [PR2 manual chapter 14: basestation setup and pairing](https://pr2s.clearpathrobotics.com/wiki/PR2%20Manual/Chapter14#Basestation_Setup_and_Pairing)
- [[Pr2-users] thread on factory resetting the router](http://lists.willowgarage.com/pipermail/pr2-users/2011-February/001403.html)
- [Troubleshooting the PR2](http://projects.csail.mit.edu/pr2/wiki/index.php?title=Troubleshooting_PR2#PR2_is_not_communicating_over_the_wireless)
- [Linksys WRT610N support page](http://support.linksys.com/en-us/support/routers/WRT610N)
