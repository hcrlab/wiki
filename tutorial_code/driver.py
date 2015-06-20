#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Driver:
    def __init__(self):
        rospy.Subscriber("/drive_forward", Float64, self.forward_callback)
        self.cmdPub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
        
    def forward_callback(self, data):
        twist_msg = Twist()
        twist_msg.linear.x = data.data
        self.cmdPub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('driver_node')
    Driver()
    rospy.spin()
