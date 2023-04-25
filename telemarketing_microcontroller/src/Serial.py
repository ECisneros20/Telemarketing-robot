#!/usr/bin/env python3
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from std_msgs.msg import Header, Float64MultiArray, Bool
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from math import pi
from Controller import MotorController
from ROBOTEQ import roboteq

# Publisher (18)
# /us1           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us2           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us3           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us4           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us5           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us6           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us7           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /us8           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir1           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir2           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir3           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir4           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir5           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir6           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir7           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /ir8           -   Range               -   to Main Computer                        -   Separate array of 19 measurements (1)
# /odom          -   Odometry            -   to Main Computer                        -   Separate array of 19 measurements (2)
# /emerg         -   Bool                -   to Main Computer                        -   Separate array of 19 measurements (1)

# Subscriber (2)
# /sensor_data   -   Float64MultiArray   -   from Microcontroller                    -   Array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder components, 1 bumper)
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

        # Sensors measurement ranges
        self.FIELD_OF_VIEW_US = pi/6
        self.MIN_RANGE_US = 0.20
        self.MAX_RANGE_US = 5.00
        self.FIELD_OF_VIEW_IR = 0.00
        self.MIN_RANGE_IR = 0.00
        self.MAX_RANGE_IR = 0.35
        self.seq = 0

        # Node initialization
        rospy.init_node("serial_communication")
        # Susbcribe to array with 22 measurements (8 ultrasonics, 8 infrared, 2 encoder components, 1 bumper)
        self.sub_sensor = rospy.Subscriber("/sensor_data", Float64MultiArray, self.callback_sensor)
        # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
        self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)
        # Rate set
        self.rate = rospy.Rate(10)

        # Messages to publish
        self.us1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_2"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_3"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_4"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_5"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_6"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_7"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.us8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "us_link_8"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.ir1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_2"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_3"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_4"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_5"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_6"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_7"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ir_link_8"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)

        self.odom = Odometry(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "odom"), child_frame_id = "base_link",
                             pose = PoseWithCovariance(pose = Pose(position = Point(x = 0.0, y = 0.0, z = 0.0),
                                                                   orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 0.0)),
                                                       covariance = Float64MultiArray(data = self.covariance)),
                             twist = TwistWithCovariance(twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                                                                       angular = Vector3(x = 0.0, y = 0.0, z = 0.0)),
                                                         covariance = Float64MultiArray(data = self.covariance)))

        self.emerg = Bool(data = 0)


    def callback_teleop(self, msg):

        # Convert linear x and angular z velocities into two velocities one for each wheel
        # max linear x = 0.3 m/s
        # max angular z = 1.5708 rad/s
        self.v_r = (2 * msg.linear.x + msg.angular.z * self.L) / (2 * self.R)
        self.v_l = (2 * msg.linear.x - msg.angular.z * self.L) / (2 * self.R)


    def callback_sensor(self, msg):

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

        # 8 US publishers
        self.us1.header.seq = self.seq
        self.us1.header.stamp = rospy.Time.now()
        self.us1.range = get_range(self.us1, msg[0])
        self.us2.header.seq = self.seq
        self.us2.header.stamp = rospy.Time.now()
        self.us2.range = get_range(self.us2, msg[1])
        self.us3.header.seq = self.seq
        self.us3.header.stamp = rospy.Time.now()
        self.us3.range = get_range(self.us3, msg[2])
        self.us4.header.seq = self.seq
        self.us4.header.stamp = rospy.Time.now()
        self.us4.range = get_range(self.us4, msg[3])
        self.us5.header.seq = self.seq
        self.us5.header.stamp = rospy.Time.now()
        self.us5.range = get_range(self.us5, msg[4])
        self.us6.header.seq = self.seq
        self.us6.header.stamp = rospy.Time.now()
        self.us6.range = get_range(self.us6, msg[5])
        self.us7.header.seq = self.seq
        self.us7.header.stamp = rospy.Time.now()
        self.us7.range = get_range(self.us7, msg[6])
        self.us8.header.seq = self.seq
        self.us8.header.stamp = rospy.Time.now()
        self.us8.range = get_range(self.us8, msg[7])

        # 8 IR publishers
        self.ir1.header.seq = self.seq
        self.ir1.header.stamp = rospy.Time.now()
        self.ir1.range = get_range(self.ir1, msg[8])
        self.ir2.header.seq = self.seq
        self.ir2.header.stamp = rospy.Time.now()
        self.ir2.range = get_range(self.ir2, msg[9])
        self.ir3.header.seq = self.seq
        self.ir3.header.stamp = rospy.Time.now()
        self.ir3.range = get_range(self.ir3, msg[10])
        self.ir4.header.seq = self.seq
        self.ir4.header.stamp = rospy.Time.now()
        self.ir4.range = get_range(self.ir4, msg[11])
        self.ir5.header.seq = self.seq
        self.ir5.header.stamp = rospy.Time.now()
        self.ir5.range = get_range(self.ir5, msg[12])
        self.ir6.header.seq = self.seq
        self.ir6.header.stamp = rospy.Time.now()
        self.ir6.range = get_range(self.ir6, msg[13])
        self.ir7.header.seq = self.seq
        self.ir7.header.stamp = rospy.Time.now()
        self.ir7.range = get_range(self.ir7, msg[14])
        self.ir8.header.seq = self.seq
        self.ir8.header.stamp = rospy.Time.now()
        self.ir8.range = get_range(self.ir8, msg[15])

        left_ticks = int(msg[16])
        right_ticks = int(msg[17])
        self.move(left_ticks, right_ticks)

        # Odom publisher
        self.odom.header.seq = self.seq
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position = [x, y, 0]
        self.odom.pose.pose.orientation = [0, 0, z, w]
        self.odom.pose.covariance = 
        self.odom.twist.twist.linear = [x, 0, 0]
        self.odom.twist.twist.angular = [0, 0, z]
        self.odom.twist.covariance = 

        # Emergency publisher
        self.emerg.data = msg[21]


    def move(self, left_ticks, right_ticks):
        control = MotorController()
        PWM_Output_left, PWM_Output_right = control.calculate_pwm_values(self.v_l, self.v_r, left_ticks, right_ticks)
        driver = roboteq(serialPort = "COM17")
        driver.setMotors("A", int(PWM_Output_left), True); driver.setMotors("B", int(PWM_Output_right), True)


    def publisherFunctions(self):        
        # Publish the array of 19 values in 18 different topics
        pub_us1 = rospy.Publisher("/us1", Range, queue_size = 10)
        pub_us2 = rospy.Publisher("/us2", Range, queue_size = 10)
        pub_us3 = rospy.Publisher("/us3", Range, queue_size = 10)
        pub_us4 = rospy.Publisher("/us4", Range, queue_size = 10)
        pub_us5 = rospy.Publisher("/us5", Range, queue_size = 10)
        pub_us6 = rospy.Publisher("/us6", Range, queue_size = 10)
        pub_us7 = rospy.Publisher("/us7", Range, queue_size = 10)
        pub_us8 = rospy.Publisher("/us8", Range, queue_size = 10)
        pub_ir1 = rospy.Publisher("/ir1", Range, queue_size = 10)
        pub_ir2 = rospy.Publisher("/ir2", Range, queue_size = 10)
        pub_ir3 = rospy.Publisher("/ir3", Range, queue_size = 10)
        pub_ir4 = rospy.Publisher("/ir4", Range, queue_size = 10)
        pub_ir5 = rospy.Publisher("/ir5", Range, queue_size = 10)
        pub_ir6 = rospy.Publisher("/ir6", Range, queue_size = 10)
        pub_ir7 = rospy.Publisher("/ir7", Range, queue_size = 10)
        pub_ir8 = rospy.Publisher("/ir8", Range, queue_size = 10)
        pub_odom = rospy.Publisher("/odom", Odometry, queue_size = 10)
        pub_emerg = rospy.Publisher("/emerg", Bool, queue_size = 10)

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub_us1.publish(self.us1)
            pub_us2.publish(self.us2)
            pub_us3.publish(self.us3)
            pub_us4.publish(self.us4)
            pub_us5.publish(self.us5)
            pub_us6.publish(self.us6)
            pub_us7.publish(self.us7)
            pub_us8.publish(self.us8)
            pub_ir1.publish(self.ir1)
            pub_ir2.publish(self.ir2)
            pub_ir3.publish(self.ir3)
            pub_ir4.publish(self.ir4)
            pub_ir5.publish(self.ir5)
            pub_ir6.publish(self.ir6)
            pub_ir7.publish(self.ir7)
            pub_ir8.publish(self.ir8)
            pub_odom.publish(self.odom)
            pub_emerg.publish(self.emerg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        com = SerialCom()
        com.publisherFunctions()
    except rospy.ROSInterruptException:
        pass
