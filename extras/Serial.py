#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from std_msgs.msg import Header, Float32MultiArray, Bool
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from math import pi
#from Controller import MotorController
#from ROBOTEQ import roboteq

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

        # Sensors measurement ranges
        self.FIELD_OF_VIEW_US = pi/6
        self.MIN_RANGE_US = 0.20
        self.MAX_RANGE_US = 5.00
        self.FIELD_OF_VIEW_IR = 0.00
        self.MIN_RANGE_IR = 0.00
        self.MAX_RANGE_IR = 0.35
        self.seq = 0


        rospy.init_node("serial_communication", anonymous=True)
        
        # Susbcribe to array with 22 measurements (8 ultrasonics, 8 infrared, 2 encoder components, 1 bumper)
        self.sub_sensor = rospy.Subscriber("/sensor_data", Float32MultiArray, self.callback_sensor)
        # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
        #self.sub_teleop = rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)
        # Rate set
        #self.rate = rospy.Rate(10)ultrasonido1_1
        
        # Messages to publish
        self.sonar_1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido1_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido2_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido3_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido4_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido5_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido6_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido7_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.sonar_8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido8_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = 0)
        self.ir_1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo1_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo2_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo3_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo4_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo5_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo6_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo7_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)
        self.ir_8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo8_1"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = 0)

        self.odom = Odometry(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "odom"), child_frame_id = "base_link",
                             pose = PoseWithCovariance(pose = Pose(position = Point(x = 0.0, y = 0.0, z = 0.0),
                                                                   orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 0.0)),
                                                       covariance = Float32MultiArray(data = self.covariance)),
                             twist = TwistWithCovariance(twist = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                                                                       angular = Vector3(x = 0.0, y = 0.0, z = 0.0)),
                                                         covariance = Float32MultiArray(data = self.covariance)))
        
        self.emerg = Bool(data = 0)
        print("ksajdkasjd ")

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

        # 8 sonar_ publishers
        self.sonar_1.header.seq = self.seq
        self.sonar_1.header.stamp = rospy.Time.now()
        self.sonar_1.range = get_range(self.sonar_1, msg.data[0])
        self.sonar_2.header.seq = self.seq
        self.sonar_2.header.stamp = rospy.Time.now()
        self.sonar_2.range = get_range(self.sonar_2, msg.data[1])
        self.sonar_3.header.seq = self.seq
        self.sonar_3.header.stamp = rospy.Time.now()
        self.sonar_3.range = get_range(self.sonar_3, msg.data[2])
        self.sonar_4.header.seq = self.seq
        self.sonar_4.header.stamp = rospy.Time.now()
        self.sonar_4.range = get_range(self.sonar_4, msg.data[3])
        self.sonar_5.header.seq = self.seq
        self.sonar_5.header.stamp = rospy.Time.now()
        self.sonar_5.range = get_range(self.sonar_5, msg.data[4])
        self.sonar_6.header.seq = self.seq
        self.sonar_6.header.stamp = rospy.Time.now()
        self.sonar_6.range = get_range(self.sonar_6, msg.data[5])
        self.sonar_7.header.seq = self.seq
        self.sonar_7.header.stamp = rospy.Time.now()
        self.sonar_7.range = get_range(self.sonar_7, msg.data[6])
        self.sonar_8.header.seq = self.seq
        self.sonar_8.header.stamp = rospy.Time.now()
        self.sonar_8.range = get_range(self.sonar_8, msg.data[7])

        # 8 ir_ publishers
        self.ir_1.header.seq = self.seq
        self.ir_1.header.stamp = rospy.Time.now()
        self.ir_1.range = get_range(self.ir_1, msg.data[8])
        self.ir_2.header.seq = self.seq
        self.ir_2.header.stamp = rospy.Time.now()
        self.ir_2.range = get_range(self.ir_2, msg.data[9])
        self.ir_3.header.seq = self.seq
        self.ir_3.header.stamp = rospy.Time.now()
        self.ir_3.range = get_range(self.ir_3, msg.data[10])
        self.ir_4.header.seq = self.seq
        self.ir_4.header.stamp = rospy.Time.now()
        self.ir_4.range = get_range(self.ir_4, msg.data[11])
        self.ir_5.header.seq = self.seq
        self.ir_5.header.stamp = rospy.Time.now()
        self.ir_5.range = get_range(self.ir_5, msg.data[12])
        self.ir_6.header.seq = self.seq
        self.ir_6.header.stamp = rospy.Time.now()
        self.ir_6.range = get_range(self.ir_6, msg.data[13])
        self.ir_7.header.seq = self.seq
        self.ir_7.header.stamp = rospy.Time.now()
        self.ir_7.range = get_range(self.ir_7, msg.data[14])
        self.ir_8.header.seq = self.seq
        self.ir_8.header.stamp = rospy.Time.now()
        self.ir_8.range = get_range(self.ir_8, msg.data[15])

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
        #self.emerg.data = msg.data[21]


    def move(self, left_ticks, right_ticks):
        control = MotorController()
        PWM_Output_left, PWM_Output_right = control.calculate_pwm_values(self.v_l, self.v_r, left_ticks, right_ticks)
        driver = roboteq(serialPort = "COM17")
        driver.setMotors("A", int(PWM_Output_left), True); driver.setMotors("B", int(PWM_Output_right), True)


    def publisherFunctions(self):        
        # Publish the array of 19 values in 18 different topics
        pub_sonar_1 = rospy.Publisher("/sonar_1", Range, queue_size = 10)
        pub_sonar_2 = rospy.Publisher("/sonar_2", Range, queue_size = 10)
        pub_sonar_3 = rospy.Publisher("/sonar_3", Range, queue_size = 10)
        pub_sonar_4 = rospy.Publisher("/sonar_4", Range, queue_size = 10)
        pub_sonar_5 = rospy.Publisher("/sonar_5", Range, queue_size = 10)
        pub_sonar_6 = rospy.Publisher("/sonar_6", Range, queue_size = 10)
        pub_sonar_7 = rospy.Publisher("/sonar_7", Range, queue_size = 10)
        pub_sonar_8 = rospy.Publisher("/sonar_8", Range, queue_size = 10)
        pub_ir_1 = rospy.Publisher("/ir_1", Range, queue_size = 10)
        pub_ir_2 = rospy.Publisher("/ir_2", Range, queue_size = 10)
        pub_ir_3 = rospy.Publisher("/ir_3", Range, queue_size = 10)
        pub_ir_4 = rospy.Publisher("/ir_4", Range, queue_size = 10)
        pub_ir_5 = rospy.Publisher("/ir_5", Range, queue_size = 10)
        pub_ir_6 = rospy.Publisher("/ir_6", Range, queue_size = 10)
        pub_ir_7 = rospy.Publisher("/ir_7", Range, queue_size = 10)
        pub_ir_8 = rospy.Publisher("/ir_8", Range, queue_size = 10)
        #pub_odom = rospy.Publisher("/odom", Odometry, queue_size = 10)
        #pub_emerg = rospy.Publisher("/emerg", Bool, queue_size = 10)

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub_sonar_1.publish(self.sonar_1)
            pub_sonar_2.publish(self.sonar_2)
            pub_sonar_3.publish(self.sonar_3)
            pub_sonar_4.publish(self.sonar_4)
            pub_sonar_5.publish(self.sonar_5)
            pub_sonar_6.publish(self.sonar_6)
            pub_sonar_7.publish(self.sonar_7)
            pub_sonar_8.publish(self.sonar_8)
            pub_ir_1.publish(self.ir_1)
            pub_ir_2.publish(self.ir_2)
            pub_ir_3.publish(self.ir_3)
            pub_ir_4.publish(self.ir_4)
            pub_ir_5.publish(self.ir_5)
            pub_ir_6.publish(self.ir_6)
            pub_ir_7.publish(self.ir_7)
            pub_ir_8.publish(self.ir_8)
            #pub_odom.publish(self.odom)
            #pub_emerg.publish(self.emerg)
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
