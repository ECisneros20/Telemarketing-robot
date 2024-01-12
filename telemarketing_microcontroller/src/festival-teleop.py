#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
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
        self.pub_servo_vel = rospy.Publisher("/servo_vel_controlled", Float32MultiArray, queue_size = 10)
        #self.pub_vel_setpoint = rospy.Publisher("/vel_setpoint", Float32MultiArray, queue_size = 10)
        self.servo_msg = Float32MultiArray(data = [0.0, 0.0])
        #self.setpoint_msg = Float32MultiArray(data = [0.0, 0.0])
        self.rate = rospy.Rate(10)

    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = -(2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)
        self.servo_msg.data = [self.v_r, self.v_l]

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_servo_vel.publish(self.servo_msg)
            rospy.loginfo(self.servo_msg)
            self.rate.sleep()

        #self.pub_servo_vel.publish(Int32MultiArray(data = [1500, 1500]))
        self.servo_msg = Float32MultiArray(data = [0.0, 0.0])

if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComTeleop()
        com.publisherFunctions()

    except rospy.ROSInterruptException:
        pass
