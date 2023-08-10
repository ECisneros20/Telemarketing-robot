#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

# Publisher (1)
# /servo_vel_controlled     -   Int32MultiArray     -   to festival-controller.py     -   Array of two motor velocities with control

# Subscriber (1)
# /servo_vel                -   Int32MultiArray     -   from festival-teleop.py       -   Array of two motor velocities without control
# /encoder_data             -   Int32MultiArray     -   from microcontroller          -   Array of two motor encoder counters

class SerialComController:

    def __init__(self):

        # Controller constants
        self.P = 0.1016
        self.I = 0.3219
        self.D = 0.0
        self.ticksR = 0
        self.ticksL = 0

        # ROS setup
        rospy.init_node("serial_com_controller_node")
        self.sub_servo_vel = rospy.Subscriber("/servo_vel", Int32MultiArray, self.callback_servo_vel)
        self.sub_encoder = rospy.Subscriber("/encoer_data", Int32MultiArray, self.callback_encoder)
        self.pub_servo_vel_control = rospy.Publisher("/servo_vel_controlled", Int32MultiArray, queue_size = 10)
        self.servo_controlled_msg = Int32MultiArray()
        self.rate = rospy.Rate(10)

    def callback_servo_vel(self, msg):

        self.servoR, self.servoL = msg.data

    def callback_encoder(self, msg):

        self.ticksR, self.ticksL = msg.data

        self.servo_controlled_msg.data = {0, 0}

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_servo_vel_control.publish(self.servo_controlled_msg)
            rospy.loginfo("Executing!")
            self.rate.sleep()

        self.pub_servo_vel_control.publish(Int32MultiArray(data=[0, 0]))


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComController()
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
