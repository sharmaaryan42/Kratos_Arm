#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32,Float32

class ButtonDetector:
    def __init__(self):
        rospy.init_node('pwm', anonymous=True)
        self.chatter_pub = rospy.Publisher('/chatter', Float32, queue_size=10)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, joy):
        msg = Float32()
        msg.data = joy
        self.chatter_pub.publish(msg)

if __name__ == '__main__':
    try:
        detector = ButtonDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass