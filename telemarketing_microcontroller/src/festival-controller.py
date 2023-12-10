#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray

from simple_pid import PID

# Publisher (1)
# /servo_vel_controlled     -   Int32MultiArray       -   to microcontroller          -   Array of two motor velocities with control

# Subscriber (2)
# /vel_setpoint             -   Float32MultiArray     -   from festival-teleop.py     -   Array of two motor velocity setpoints
# /encoder_data             -   Float32MultiArray     -   from microcontroller        -   Array of two motor angular velocities

class SerialComController:

    def __init__(self):

        # Controller constants
        self.Kp_r = 4.0
        self.Ki_r = 0.0
        self.Kd_r = 0.0
        self.Kp_l = 4.0
        self.Ki_l = 0.0
        self.Kd_l = 0.0
        self.setpointR = 0
        self.setpointL = 0
        self.velRightWheel = 0
        self.velLeftWheel = 0

        # ROS setup
        rospy.init_node("serial_com_controller_node")
        self.sub_vel_setpoint = rospy.Subscriber("/vel_setpoint", Float32MultiArray, self.callback_vel_setpoint)
        self.sub_encoder = rospy.Subscriber("/encoder_data", Float32MultiArray, self.callback_encoder)
        self.pub_servo_vel_control = rospy.Publisher("/servo_vel_controlled", Int32MultiArray, queue_size = 10)
        self.servo_controlled_msg = Int32MultiArray(data = [1500, 1500])
        self.rate = rospy.Rate(10)


    def calculate_servo_velocity(self):

        pid_R = PID(self.Kp_r, self.Ki_r, self.Kd_r, setpoint = self.setpointR)
        pid_L = PID(self.Kp_l, self.Ki_l, self.Kd_l, setpoint = self.setpointL)
        pid_R.output_limits = (-300, 300)
        pid_L.output_limits = (-300, 300)
        pid_R.sample_time = 0.001
        pid_L.sample_time = 0.001

        return [pid_R(self.velRightWheel)+1500, pid_L(self.velLeftWheel)+1500]

    def callback_vel_setpoint(self, msg):

        self.setpointR, self.setpointL = msg.data

    def callback_encoder(self, msg):

        self.velRightWheel, self.velLeftWheel = msg.data
        rospy.loginfo(str(self.velRightWheel) + str(self.velLeftWheel))
        self.servo_controlled_msg.data = self.calculate_servo_velocity()

    def publisherFunctions(self):

        while not rospy.is_shutdown():
            self.pub_servo_vel_control.publish(self.servo_controlled_msg)
            rospy.loginfo("Executing!")
            self.rate.sleep()

        self.pub_servo_vel_control.publish(Int32MultiArray(data = [1500, 1500]))


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComController()
        com.publisherFunctions()

    except rospy.ROSInterruptException:
        pass
