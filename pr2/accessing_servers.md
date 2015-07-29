# Accessing Servers

**Note: This is a potentially dangerous process. Doing this improperly (for example, though creating an electrical short) can cause harm to you and damage the robot.**

You may at some point want to access or remove the servers on the PR2. For example if you want to connect a monitor or keyboard to c1 or c2. There are remote KVM instructions but I think the general consensus on the Pr2-users mailing list is that those don't work very well. 

**IMPORTANT:** Always use the screwdriver and bits included in the PR2 toolkit. They are very high quality and will help prevent stripping some of the more delicate screws.

<img src="https://raw.githubusercontent.com/hcrlab/wiki/master/pr2/img/tool_kit_highlight.png" width="50%" height="50%" />

Here's what we learned from taking apart the PR2 the last time. If you have any questions ask Leah or Sarah. 
All the instructions for working with specific parts of the PR2 hardware can be found [here](https://pr2s.clearpathrobotics.com/wiki/PR2%20Service%20Information). (At the time of writing this, you might need to access a cached copy, but all the links on the cached page still work. One link that might not work is the one to the instructions for [replacing the CMOS battery](http://www.clearpathrobotics.com/wp-content/uploads/2015/07/PR2-Server-CMOS-Battery-Replacement.pdf).).

1. To access the servers, if possible, you should raise the torso of the PR2 before you start. Otherwise, you can find out how to raise the torso manually [here](http://www.clearpathrobotics.com/wp-content/uploads/2014/07/34939536-PR2-Manual-Spine-Movement.pdf).
2. Then you should remove the [metal base skirts](http://www.clearpathrobotics.com/wp-content/uploads/2014/07/34742457-PR2-Metal-Base-Skirts-Removal-and-Replacement.pdf) and the [white plastic base panel and back panel](http://www.clearpathrobotics.com/wp-content/uploads/2014/07/34742421-PR2-Bottom-Front-and-Bottom-Rear-Panel-Removal-and-Replacement.pdf). 
3. Then you probably have to remove the metal trim (just the middle one) as described [here](http://www.clearpathrobotics.com/wp-content/uploads/2014/07/34939478-PR2-Trim-Panels-Removal-and-Replacement.pdf). 
4. Next remove the [bellows](http://www.clearpathrobotics.com/wp-content/uploads/2014/07/34939399-PR2-Bellows-Removal-and-Replacement.pdf).
5. Now you can remove the [servers](https://pr2s.clearpathrobotics.com/wiki/PR2%20Service%20Information). (At the time of writing this, you might need to access a cached copy, but all the links on the cached page still work. One link that might not work is the one to the instructions for [replacing the CMOS battery](http://www.clearpathrobotics.com/wp-content/uploads/2015/07/PR2-Server-CMOS-Battery-Replacement.pdf). There is a free USB port on the back of c1 for a keyboard and an accessible VGA port for a monitor. On c2 there is a DVI port that's easily accessible for a monitor. There are no free USB ports, so the easiest thing to do is unplug the USB extender from the Bluetooth module on top of c2 and plug a keyboard into the extender. 


