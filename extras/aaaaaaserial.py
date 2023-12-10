#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from std_msgs.msg import Float64MultiArray, Bool, Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, 
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from math import pi

# Publisher (19)
# /servo_vel     -   Float64MultiArray   -   to Microcontroller                      -   Convert from velocities to two-array of servo equivalents (e.g. 1500 ms)
# /us1           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us2           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us3           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us4           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us5           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us6           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us7           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /us8           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir1           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir2           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir3           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir4           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir5           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir6           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir7           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /ir8           -   Range               -   to Main Computer                        -   Separate array of 22 sensors (1)
# /odom          -   Odometry            -   to Main Computer                        -   Separate array of 22 sensors (5)
# /emerg         -   Bool                -   to Main Computer                        -   Separate array of 22 sensors (1)

# Subscriber (2)
# /sensor_data   -   Float64MultiArray   -   from Microcontroller                    -   Array with 22 measurements (8 ultrasonics, 8 infrared, 5 encoder components, 1 bumper)
# /cmd_vel       -   Twist               -   from Main computer or PS4 controller    -   Linear velocity x and angular velocity z

# Robot constants
R = 0.1016 # Wheel radius (m)
L = 0.3219 # Distance between the wheels (m)
v_r = 0.0 # Angular velocity for right motor
v_l = 0.0 # Angular velocity for left motor
covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
# Sensors' measurement ranges
FIELD_OF_VIEW_US = pi/6
MIN_RANGE_US = 0.20
MAX_RANGE_US = 5.00
FIELD_OF_VIEW_IR = 0.00
MIN_RANGE_IR = 0.00
MAX_RANGE_IR = 0.35
seq = 0

# Messages to publish
# Include Float64MultiArray(data = ) 
servo_vel = Float64MultiArray(data = [0.0, 0.0])
# Include Range(header = Header(seq = , stamp = ), range = )
us1 = Range(header = Header(frame_id = "us_link_1"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us2 = Range(header = Header(frame_id = "us_link_2"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us3 = Range(header = Header(frame_id = "us_link_3"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us4 = Range(header = Header(frame_id = "us_link_4"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us5 = Range(header = Header(frame_id = "us_link_5"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us6 = Range(header = Header(frame_id = "us_link_6"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us7 = Range(header = Header(frame_id = "us_link_7"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
us8 = Range(header = Header(frame_id = "us_link_8"), radiation_type = 0, field_of_view = FIELD_OF_VIEW_US, min_range = MIN_RANGE_US, max_range = MAX_RANGE_US)
ir1 = Range(header = Header(frame_id = "ir_link_1"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir2 = Range(header = Header(frame_id = "ir_link_2"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir3 = Range(header = Header(frame_id = "ir_link_3"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir4 = Range(header = Header(frame_id = "ir_link_4"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir5 = Range(header = Header(frame_id = "ir_link_5"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir6 = Range(header = Header(frame_id = "ir_link_6"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir7 = Range(header = Header(frame_id = "ir_link_7"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
ir8 = Range(header = Header(frame_id = "ir_link_8"), radiation_type = 1, field_of_view = FIELD_OF_VIEW_IR, min_range = MIN_RANGE_IR, max_range = MAX_RANGE_IR)
# Include Odometry(header = Header(seq = , stamp = ), pose = PoseWithCovariance(Pose = (position, orientation)), twist = TwistWithCovariance(Twist = (linear, angular)))
odom = Odometry(header = Header(frame_id = "odom"), child_frame_id = "base_link", pose = PoseWithCovariance(covariance = Float64MultiArray(data = covariance)), twist = TwistWithCovariance(covariance = Float64MultiArray(data = covariance)))
# Include Bool(data = )
emerg = Bool(data = 0)


def callback_teleop(msg):

    global R, L, v_r, v_l

    # Convert linear x and angular z velocities into two velocities one for each wheel
    # max linear x = 0.3 m/s
    # max angular z = 1.5708 rad/s
    v_r_aux = (2 * msg.linear.x + msg.angular.z * L) / (2 * R)
    v_l_aux = (2 * msg.linear.x - msg.angular.z * L) / (2 * R)

    # These two velocities are converted into two RC values (e.g. 1500 ms)
    # TODO: Compensation to be refined.
    # TODO: Assume temporarily that as peak rad/s for only linear and only angular are similar, it does not need any compensation
    # TODO: Assume 1500 ms neutral and peaks are +/- 150 ms -> 1650 ms = 2.95 rad/s. Also that possitive rotation is counterclockwise seeing the motor axis
    '''
    L : 1650 (+) 2.95 rad/s | 1350 (-) -2.95 rad/s
    R : 1350 (+) 2.95 rad/s | 1600 (-) -2.95 rad/s
    L = (3000/59)X + 1500
    R = -(3000/59)X + 1500
    '''
    v_r = (3000/59)*v_r_aux + 1500
    v_l = -(3000/59)*v_l_aux + 1500


def callback_sensor(msg):

    def get_range(range_msg, range):
        if range < range_msg.min_range:
            return float("-Inf")
        elif range > range_msg.max_range:
            return float("Inf")
        else:
            return range

    def get_odom():
        # TODO: Function that use the raw information from encoder to get the odometry
        # TODO: Define input and how to interpret
        # TODO: All the big computation must be done in the pc
        pass

    global seq

    seq += 1

    us1.header.seq = seq
    us1.header.stamp = rospy.Time.now()
    us1.range = get_range(us1, msg[0])
    us2.header.seq = seq
    us2.header.stamp = rospy.Time.now()
    us2.range = get_range(us2, msg[1])
    us3.header.seq = seq
    us3.header.stamp = rospy.Time.now()
    us3.range = get_range(us3, msg[2])
    us4.header.seq = seq
    us4.header.stamp = rospy.Time.now()
    us4.range = get_range(us4, msg[3])
    us5.header.seq = seq
    us5.header.stamp = rospy.Time.now()
    us5.range = get_range(us5, msg[4])
    us6.header.seq = seq
    us6.header.stamp = rospy.Time.now()
    us6.range = get_range(us6, msg[5])
    us7.header.seq = seq
    us7.header.stamp = rospy.Time.now()
    us7.range = get_range(us7, msg[6])
    us8.header.seq = seq
    us8.header.stamp = rospy.Time.now()
    us8.range = get_range(us8, msg[7])
    
    ir1.header.seq = seq
    ir1.header.stamp = rospy.Time.now()
    ir1.range = get_range(ir1, msg[8])
    ir2.header.seq = seq
    ir2.header.stamp = rospy.Time.now()
    ir2.range = get_range(ir2, msg[9])
    ir3.header.seq = seq
    ir3.header.stamp = rospy.Time.now()
    ir3.range = get_range(ir3, msg[10])
    ir4.header.seq = seq
    ir4.header.stamp = rospy.Time.now()
    ir4.range = get_range(ir4, msg[11])
    ir5.header.seq = seq
    ir5.header.stamp = rospy.Time.now()
    ir5.range = get_range(ir5, msg[12])
    ir6.header.seq = seq
    ir6.header.stamp = rospy.Time.now()
    ir6.range = get_range(ir6, msg[13])
    ir7.header.seq = seq
    ir7.header.stamp = rospy.Time.now()
    ir7.range = get_range(ir7, msg[14])
    ir8.header.seq = seq
    ir8.header.stamp = rospy.Time.now()
    ir8.range = get_range(ir8, msg[15])

    '''
    # TODO: Obtain pose and twist values for the Odometry messages
    enc1.header.stamp = rospy.Time.now()
    enc1.pose.pose = 
    enc1.twist.pose = 
    enc2.header.stamp = rospy.Time.now()
    enc2.pose.pose = 
    enc2.twist.pose = 
    
    emerg = Bool(data = msg[21])
    '''

def serial_communication():

    rospy.init_node("serial_communication")

    # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
    sub_teleop = rospy.Subscriber("/cmd_vel", Twist, callback_teleop)
    # Publish array with the two servo equivalents to the linear and angular velocities
    pub_teleop = rospy.Publisher("/servo_vel", Float64MultiArray, queue_size = 10)

    # Susbcribe to array with 22 measurements (8 ultrasonics, 8 infrared, 5 encoder components, 1 bumper)
    sub_sensor = rospy.Subscriber("/sensor_data", Float64MultiArray, callback_sensor)
    # Publish the array of 22 values in 18 different topics
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
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        servo_vel.data = v_l, v_r
        pub_teleop.publish(servo_vel)
        pub_us1.publish(us1)
        pub_us2.publish(us2)
        pub_us3.publish(us3)
        pub_us4.publish(us4)
        pub_us5.publish(us5)
        pub_us6.publish(us6)
        pub_us7.publish(us7)
        pub_us8.publish(us8)
        pub_ir1.publish(ir1)
        pub_ir2.publish(ir2)
        pub_ir3.publish(ir3)
        pub_ir4.publish(ir4)
        pub_ir5.publish(ir5)
        pub_ir6.publish(ir6)
        pub_ir7.publish(ir7)
        pub_ir8.publish(ir8)
        pub_odom.publish(odom)
        pub_emerg.publish(emerg)
        rate.sleep()


if __name__ == "__main__":
    try:
        serial_communication()
    except rospy.ROSInterruptException:
        pass