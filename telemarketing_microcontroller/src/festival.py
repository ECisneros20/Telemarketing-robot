#!/usr/bin/env python
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from std_msgs.msg import Int16MultiArray, Twist
from math import pi

# Publisher (1)
# /servo_vel     -   Int16MultiArray     -   to Microcontroller      -   Array of two motor velocities

# Subscriber (1)
# /cmd_vel       -   Twist               -   from PS4 controller     -   Linear velocity x and angular velocity z

class SerialComSensors:

    def __init__(self):

        # Robot constants
        self.R = 0.1016 # Wheel radius (m)
        self.L = 0.3219 # Distance between the wheels (m)
        self.v_r = 0.0 # Angular velocity for right motor
        self.v_l = 0.0 # Angular velocity for left motor

        rospy.init_node("serial_communication_teleop", anonymous = False)

        # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
        self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)
        # Publish the array of motor velocities
        self.pub_servo_vel = rospy.Publisher("/servo_vel", Int16MultiArray, queue_size = 10)

        # Messages to publish
        self.servo_msg = Int16MultiArray()

    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = -(2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)
        self.servo_msg.data = {self.v_r, self.v_l}

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_servo_vel.publish(self.servo_vel)
            rospy.loginfo("Executing!")
            self.rate.sleep()


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComSensors()
        rate = rospy.Rate(10)
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
