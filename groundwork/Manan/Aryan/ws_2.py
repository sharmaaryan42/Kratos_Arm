#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
if __name__ == '__main__':
    rospy.init_node('motor')
    pub = rospy.Publisher("/chatter", Int32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        num = int(input("Please enter your number."))
        pub.publish(num)
        rate.sleep()
   