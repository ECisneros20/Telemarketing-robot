#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

# Publisher (1)
# /servo_vel     -   Int32MultiArray     -   to festival-controller.py     -   Array of two motor velocities without control

# Subscriber (1)
# /cmd_vel       -   Twist               -   from PS4 controller           -   Linear velocity x and angular velocity z

class SerialComTeleop:

    def __init__(self):

        # Robot constants
        self.R = 0.1016 # Wheel radius (m)
        self.L = 0.3219 # Distance between the wheels (m)
        self.v_r = 0.0 # Angular velocity for right motor
        self.v_l = 0.0 # Angular velocity for left motor

        # ROS setup
        rospy.init_node("serial_com_teleop_node")
        self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)
        self.pub_servo_vel = rospy.Publisher("/servo_vel", Int32MultiArray, queue_size = 10)
        self.servo_msg = Int32MultiArray(data = [1500, 1500])
        self.rate = rospy.Rate(10)

    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = -(2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)
        rospy.loginfo(str(self.v_r) + str(self.v_l))
        self.servo_msg.data = [int(50.80 * self.v_r + 1500), int(50.80 * self.v_l + 1500)]

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_servo_vel.publish(self.servo_msg)
            rospy.loginfo("Executing!")
            self.rate.sleep()

        self.pub_servo_vel.publish(Int32MultiArray(data=[1500, 1500]))


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComTeleop()
        com.publisherFunctions()

    except rospy.ROSInterruptException:
        pass
