#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
import math

class ButtonDetector:
    def __init__(self):
        rospy.init_node('button_detector', anonymous=True)
        self.chatter_pub = rospy.Publisher('/chatter', Int32MultiArray, queue_size=10)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def map_float(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, joy):
        msg = Int32MultiArray()
        pwm_actuator1=0
        pwm_actuator2=0
        dir_actuator2=0
        dir_actuator1=0

        dir_gripper=0
        pwm_gripper =0

        pwm_base=0
        dir_base=0

        pwm_bevel1 = 0
        pwm_bevel2 = 0
        dir_bevel1 = 0
        dir_bevel2 = 0

        # Actuator control
        # Process input1 (Joy Axis 1)
        input1 = joy.axes[1]
        absoluteinput1 = abs(input1)
        output1 = self.map_float(absoluteinput1, 0.0, 1.0, 0.0, 255.0)
        roundedoutput1 = round(output1)
        pwm_actuator1 = roundedoutput1
        dir_actuator1 = 1 if input1 > 0 else -1 if input1 < 0 else 0

        # Process input2 (Joy Axis 0)
        input2 = joy.axes[0]
        absoluteinput2 = abs(input2)
        output2 = self.map_float(absoluteinput2, 0.0, 1.0, 0.0, 255.0)
        roundedoutput2 = round(output2)
        pwm_actuator2 = roundedoutput2
        dir_actuator2 = 1 if input2 > 0 else -1 if input2 < 0 else 0

        # Bevel control
        # Take 1st axis input for pitch
        input3 = joy.axes[3]
        absoluteinput3 = abs(input3)
        output3 = self.map_float(absoluteinput3, 0.0, 1.0, 0.0, 255.0)
        roundedoutput3 = round(output3)

        # Take 2nd axis input for roll
        input4 = joy.axes[4]
        absoluteinput4 = abs(input4)
        output4 = self.map_float(absoluteinput4, 0.0, 1.0, 0.0, 255.0)
        roundedoutput4 = round(output4)

        # Combine
        if absoluteinput3 > absoluteinput4:
            pwm_bevel1 = pwm_bevel2 = roundedoutput3
            dir_bevel1 = dir_bevel2 = 1 if input3 > 0 else -1
        else:
            pwm_bevel1 = pwm_bevel2 = roundedoutput4
            if input4 > 0:
                dir_bevel1, dir_bevel2 = -1, 1
            else:
                dir_bevel1, dir_bevel2 = 1, -1

        # Gripper control
        if joy.buttons[1] == 1:
            pwm_gripper = 200
            dir_gripper = 1
        elif joy.buttons[3] == 1:
            pwm_grippel = 200
            dir_gripper = -1
        else:
            pwm_gripper = 0
            dir_gripper = 0 

        # Base control
        input5 = joy.axes[2]
        output5 = self.map_float(input5, 1.0, -1.0, 0.0, 255.0)
        roundedoutput5 = round(output5)

        input6 = joy.axes[5]
        output6 = self.map_float(input6, 1.0, -1.0, 0.0, 255.0)
        roundedoutput6 = round(output6)

        if output5 > output6:
            dir_base = 1
            pwm_base = roundedoutput5
        elif output5 < output6:
           dir_base = -1
           pwm_base = roundedoutput6
        else:
           pwm_base = 0

        # Fill the message with calculated PWM and direction values
        msg.data = [
            pwm_actuator1, dir_actuator1, pwm_actuator2, dir_actuator2,
            pwm_bevel1, dir_bevel1, pwm_bevel2, dir_bevel2,
            pwm_gripper, dir_gripper,
            pwm_base, dir_base
        ]
        self.chatter_pub.publish(msg)

if __name__ == '__main__':
    try:
        detector = ButtonDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
