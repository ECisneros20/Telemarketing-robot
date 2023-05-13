#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45

import rospy
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

# Publisher (1)
# /odom          -   Odometry            -   to Main Computer                        -   Separate array of 19 measurements (2)

# Subscriber (2)
# /sensor_data   -   Float32MultiArray   -   from Microcontroller                    -   Array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder components, 1 bumper)
# /cmd_vel       -   Twist               -   from Main computer or PS4 controller    -   Linear velocity x and angular velocity z

class SerialCom:

    def __init__(self):

        # Robot constants
        self.R = 0.1016 # Wheel radius (m)
        self.L = 0.3219 # Distance between the wheels (m)
        self.v_r = 0.0 # Angular velocity for right motor
        self.v_l = 0.0 # Angular velocity for left motor
        self.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        self.seq = 0

        rospy.init_node("serial_communication_controller", anonymous = True)

        # Susbcribe to array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder components, 1 bumper)
        self.sub_sensor = rospy.Subscriber("/sensor_data", Float32MultiArray, self.callback_sensor)
        # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
        self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)

        # Messages to publish
        self.odom = Odometry(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "odom"), child_frame_id = "base_link",
                             pose = PoseWithCovariance(pose = Pose(position = Point(x = 0.0, y = 0.0, z = 0.0),
                                                                   orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 0.0)),
                                                       covariance = Float32MultiArray(data = self.covariance)),
                             twist = TwistWithCovariance(twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                                                                       angular = Vector3(x = 0.0, y = 0.0, z = 0.0)),
                                                         covariance = Float32MultiArray(data = self.covariance)))


    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = (2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)


    def callback_sensor(self, msg):
        print(msg)
        def get_range(range_msg, range):

            if range < range_msg.min_range:
                return float("-Inf")
            elif range > range_msg.max_range:
                return float("Inf")
            else:
                return range

        def get_quaternion():

            pass

        self.seq += 1

        #left_ticks = int(msg.data[16])
        #right_ticks = int(msg.data[17])
        #self.move(left_ticks, right_ticks)
        #print("US: ", self.us1.range, "-", self.us2.range, "-", self.us3.range, "-", self.us4.range, "-", self.us5.range, "-", self.us6.range, "-", self.us7.range, "-", self.us8.range, "-")
        #print("IR: ", self.ir1.range, "-", self.ir2.range, "-", self.ir3.range, "-", self.ir4.range, "-", self.ir5.range, "-", self.ir6.range, "-", self.ir7.range, "-", self.ir8.range, "-")
        # # Odom publisher
        # self.odom.header.seq = self.seq
        # self.odom.header.stamp = rospy.Time.now()
        # self.odom.pose.pose.position = [x, y, 0]
        # self.odom.pose.pose.orientation = [0, 0, z, w]
        # self.odom.pose.covariance = 0
        # self.odom.twist.twist.linear = [x, 0, 0]
        # self.odom.twist.twist.angular = [0, 0, z]
        # self.odom.twist.covariance = 0

        # Emergency publisher


    def move(self, left_ticks, right_ticks):
        control = MotorController()
        PWM_Output_left, PWM_Output_right = control.calculate_pwm_values(self.v_l, self.v_r, left_ticks, right_ticks)
        driver = roboteq(serialPort = "COM17")
        driver.setMotors("A", int(PWM_Output_left), True); driver.setMotors("B", int(PWM_Output_right), True)


    def publisherFunctions(self):
        # Publish the array of 19 values in 18 different topics
        pub_odom = rospy.Publisher("/odom", Odometry, queue_size = 10)

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub_odom.publish(self.odom)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        # Node initialization
        com = SerialCom()
        rate = rospy.Rate(10)
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
