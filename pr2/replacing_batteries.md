# Replacing the PR2 batteries

**Note: This is a potentially dangerous process. Doing this improperly (for example, though creating an electrical short) can cause harm to you and damage the robot.**

The PR2 uses 16 lithium ion batteries from [Ocean Server](http://www.ocean-server.com/). The PR2 Tool Kit provides tools of the correct sizes to work on the robot.

There are four cages with four batteries each. They are located at the base of the robot, with two cages on the left and two cages on the right. The cages are attached to the chassis with four screws. Unscrew them. The batteries are held in with two zip ties. Cut and remove them.

Each cage holds three things:

1. The batteries themselves.
2. A battery controller.
3. A power supply.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/battery_1.jpg" width="50%" height="50%" />

This photo shows the two cages on the right side of the robot. The screws holding the cages into the chassis have already been removed, and the zip ties for the top cage have already been cut. The batteries are the black rectangles. The power supplies are on the left, with the fans on them. The battery controller is in between, with all the wires plugged into it.

The cages slide out. There are three wires connecting the cages to the rest of the robot:

1. "AC Pwr Batt Module": This connects to a large, 3-pin connector on the power supply.
2. "Batt Module Stackup": This is an 2x4 pin connector.
3. A ribbon cable: For the cages on the left, you can just disconnect the cable from the robot. For the cages on the right, the ribbon cable runs under the chassis to the left side of the robot, and is very difficult to unplug. Instead of disconnecting the cable, you can just slide the battery cage out and still have enough room to work.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/battery_2.jpg" width="50%" height="50%" />

The "AC Pwr Batt Module" cable is the black cable labeled "Top Right", which has a black, tan, and green cable coming from it. The "Batt Module Stackup" cable for the top battery cage is the twisted orange and red cable. The bottom battery cage's "Batt Module Stackup" cable is twisted orange and white. You can also see the two ribbon cables.

The batteries slide out of the cages. Each battery is connected to the battery controller with a six pin connector. The top two batteries are connected to the battery controller from the front, and will slide out easily once disconnected. However, the wires for the bottom two batteries are routed under the battery controller, and plugged into the back. Once you have disconnected them, it's not easy to pull the wires out.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/battery_3.jpg" width="50%" height="50%" />

In this photo, the top two batteries have been removed. The bottom two batteries have wires running under the battery controller, and can't be pulled out.

To get the batteries out, we need to unscrew the battery controller from the cage. The battery controller is attached to the cage with four screws: two on the top, and two on the bottom. In the photo above, we have already removed the two screws on top holding the battery controller in. Once the screws are removed, the battery controller should be loose inside the cage. If it's not loose enough to get the battery cables out, it may be necessary to unplug the battery controller from the power supply. This is connected with a wide, 10 pin connector next to where the "AC Pwr Batt Module" cable was plugged in.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/battery_4.jpg" width="50%" height="50%" />

It may be necessary to unplug the wide, 10-pin connector (already unplugged in this picture) so that the battery controller can clear enough space to pull out the battery cables. The battery cables are the unplugged blue/red/white/green/black cables at the bottom right, they are still stuck behind the battery controller.

Now you can finally remove the bottom two batteries.

Next, slide two new batteries in to the bottom slots. Route the cables under the battery controller, as they were before. Connect the batteries. Once you have them in place, you can plug the battery controller back into the power supply and screw the battery controller back to the battery cage.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/battery_5.jpg" width="50%" height="50%" />

Sliding in the new bottom battery. Note the orientation. Also note that the battery controller is still loose inside the cage in this photo.

Connect the "AC Pwr Batt Module" and the "Batt Module Stackup" cables, and slide the cage back into place. The top two batteries are easier to insert, since they plug into the battery controller from the front. If you disconnected the ribbon cable, reconnect it. We did not zip tie the batteries to the battery cage, but you may want to. Finally, screw the battery cage back into the chassis.

Repeat for the other battery cages.
