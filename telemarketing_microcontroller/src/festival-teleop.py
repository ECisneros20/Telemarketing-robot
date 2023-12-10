#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float64
from geometry_msgs.msg import Twist

# Publisher (2)
# /servo_vel        -   Int32MultiArray       -   to microcontroller (formerly)     -   Array of two motor velocities without control
# /vel_setpoint     -   Float32MultiArray     -   to festival-controller.py         -   Array of two motor velocity setpoints

# Subscriber (1)
# /cmd_vel          -   Twist                 -   from PS4 controller               -   Linear velocity x and angular velocity z

class SerialComTeleop:

    def __init__(self):

        # Robot constants
        self.R = 0.1016 # Wheel radius (m)
        self.L = 0.43454 # Distance between the wheels (m)
        self.v_r = 0.0 # Angular velocity for right motor
        self.v_l = 0.0 # Angular velocity for left motor

        # ROS setup
        rospy.init_node("serial_com_teleop_node")
        self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)
        self.pub_vel_setpoint_left = rospy.Publisher("/vel_setpoint_left", Float64, queue_size = 10)
        self.setpoint_left_msg = Float64(data = 0.0)
        self.pub_vel_setpoint_right = rospy.Publisher("/vel_setpoint_right", Float64, queue_size = 10)
        self.setpoint_right_msg = Float64(data = 0.0)
        self.rate = rospy.Rate(10)

    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = -(2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)
        self.setpoint_left_msg.data = self.v_l
        self.setpoint_right_msg.data = self.v_r

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_vel_setpoint_left.publish(self.setpoint_left_msg)
            self.pub_vel_setpoint_right.publish(self.setpoint_right_msg)
            rospy.loginfo(self.setpoint_left_msg)
            rospy.loginfo(self.setpoint_right_msg)
            self.rate.sleep()

        #self.pub_servo_vel.publish(Int32MultiArray(data = [1500, 1500]))
        self.setpoint_left_msg.data = 0.0
        self.setpoint_right_msg.data = 0.0

if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComTeleop()
        com.publisherFunctions()

    except rospy.ROSInterruptException:
        pass
