#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45

import rospy
from std_msgs.msg import Header, Float32MultiArray, Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

from simple_pid import PID 
import time

# Publisher (1)
# /odom          -   Odometry            -   to Main Computer                        -   Separate array of 19 measurements (2)

# Subscriber (2)
# /sensor_data   -   Float32MultiArray   -   from Microcontroller                    -   Array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder signals, 1 bumper)
# /cmd_vel       -   Twist               -   from Main computer or PS4 controller    -   Linear velocity x and angular velocity z

class SerialComController:

	def __init__(self):

		# Robot constants
		self.R = 0.1015 # Wheel radius (m)
		self.L = 0.35 # Distance between the wheels (m)
		self.v_r_target = 0.0 # Angular velocity for right motor
		self.v_l_target = 0.0 # Angular velocity for left motor
		self.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
							0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
		self.seq = 0

		#MotorController
		self.TICKS_PER_REVOLUTION = 8000 #Calculate this value manually
		self.TICKS_PER_METER = 12544 #Number of ticks a wheel makes moving a linear distance of 1 meter
		self.prevLeftCount=0
		self.prevTimeLeft=0      
		self.prevRightCount=0
		self.prevTimeRight=0
		self.left_ticks=0
		self.right_ticks=0
		self.left_pwm=0
		self.right_pwm=0
		self.vel = [0.0, 0.0]

		rospy.init_node("serial_communication_controller", anonymous = False)

		# Susbcribe to array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder signals, 1 bumper)
		self.sub_sensor = rospy.Subscriber("/sensor_data", Float32MultiArray, self.callback_sensor)
		# Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
		self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)

		# Messages to publish
		self.pwm = Float32MultiArray(data=[0.0, 0.0])


	def callback_teleop(self, msg):

		# Convert linear x and angular z velocities into two velocities one for each wheel
		# max linear x = 0.3 m/s
		# max angular z = 1.5708 rad/s
		self.v_r_target = -(2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
		self.v_l_target = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)
		self.vel = [self.v_l_target, self.v_r_target]


	def callback_sensor(self, msg):

		def get_quaternion():

			pass

		self.seq += 1

		# Control signal

		# Odometry publisher
		self.right_ticks = int(msg.data[16])
		self.left_ticks = int(msg.data[17])

	def publisherFunctions(self):

		# Publish the Odometry message
		pub_vel = rospy.Publisher("/vel_motors", Float32MultiArray, queue_size = 10)

		self.rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			self.pwm.data=self.calculate_pwm_values(self.vel[0],self.vel[1],self.left_ticks,self.right_ticks)
			pub_vel.publish(self.pwm)
			self.rate.sleep()

	#MotorCOntrollerFUnctions
	def calculate_vel_left_wheel(self, left_ticks):
		numOfTicks = (65535 + left_ticks - self.prevLeftCount) % 65535
		if (numOfTicks > 10000):#Whenever the counter pass from positive to negative values
			numOfTicks = 0 - (65535 - numOfTicks)
		self.velLeftWheel = numOfTicks/self.TICKS_PER_METER/((time.time())-self.prevTimeLeft)
		print("VelI:")
		print(self.velLeftWheel)
		self.prevLeftCount=left_ticks
		self.prevTimeLeft=time.time()
		

	def calculate_vel_right_wheel(self, right_ticks):
		numOfTicks = (65535 + right_ticks - self.prevRightCount) % 65535
		if (numOfTicks > 10000):
			numOfTicks = 0 - (65535 - numOfTicks)
		self.velRightWheel = numOfTicks/self.TICKS_PER_METER/((time.time())-self.prevTimeRight)
		print("VelD:")
		print(self.velRightWheel)
		self.prevRightCount=right_ticks
		self.prevTimeRight=time.time()

		
	def calculate_pwm_values(self, desiredSpeed_left, desiredSpeed_right, left_ticks, right_ticks):
		self.calculate_vel_left_wheel(left_ticks)
		self.calculate_vel_right_wheel(right_ticks)
		Kp_left=1;Ki_left=1;Kd_left=1
		Kp_right=1;Ki_right=1;Kd_right=1
		pid_left = PID(Kp_left, Ki_left, Kd_left, setpoint=desiredSpeed_left)
		pid_right = PID(Kp_right, Ki_right, Kd_right, setpoint=desiredSpeed_right)
		pid_left.output_limits = (-255, 255);pid_right.output_limits = (-255, 255)
		pid_left.sample_time = 0.001;pid_right.sample_time = 0.001

		PWM_Output_left = pid_left(self.velLeftWheel)
		PWM_Output_right = pid_right(self.velRightWheel)
		PWM_Output = [PWM_Output_right, PWM_Output_left]
		print("PWM")
		print(PWM_Output)
		return PWM_Output


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComController()
        rate = rospy.Rate(10)
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass