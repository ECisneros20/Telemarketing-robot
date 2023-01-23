#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float64MultiArray

# Publisher
# /servo_vel     -   Float64MultiArray   -   to Microcontroller                      -   Convert from velocities to two-array of servo equivalents (e.g. 1500 ms)
# /us1           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us2           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us3           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us4           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us5           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us6           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us7           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /us8           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir1           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir2           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir3           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir4           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir5           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir6           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir7           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /ir8           -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /enc1          -   Range               -   to Main Computer                        -   Separate array of 18 sensors
# /enc2          -   Range               -   to Main Computer                        -   Separate array of 18 sensors

# Subscriber
# /sensor_data   -   Float64MultiArray   -   from Microcontroller                    -   Array with 18 measurements (8 ultrasonics, 8 infrared, 2 encoders)
# /cmd_vel       -   Twist               -   from Main computer or PS4 controller    -   Linear velocity x and angular velocity z


# Robot set constants

R = 0.1016 # Wheel radius (m)
L = 0.3219 # Distance between the wheels (m)
vel_motor_right = 0.0 # Angular velocity for right motor
vel_motor_left = 0.0 # Angular velocity for left motor

vel_des = Float64MultiArray()
vel_des.data.append(0.0)
vel_des.data.append(0.0)

'''
# Data sensor subscription

num_sensors = 18
sensor_data = Float64MultiArray()

for i in range(num_sensors):
    sensor_data.data.append(0.0)


 Data sensor publication '''
'''
us1 = Range()
us1.header = Header()
us1.header.frame_id = 'us_link_1'

us2 = Range()
us2.header = Header()
us2.header.frame_id = 'us_link_2'

us3 = Range()
us3.header = Header()
us3.header.frame_id = 'us_link_3'

us4 = Range()
us4.header = Header()
us4.header.frame_id = 'us_link_4'

us5 = Range()
us5.header = Header()
us5.header.frame_id = 'us_link_5'

us6 = Range()
us6.header = Header()
us6.header.frame_id = 'us_link_6'

us7 = Range()
us7.header = Header()
us7.header.frame_id = 'us_link_7'

us8 = Range()
us8.header = Header()
us8.header.frame_id = 'us_link_8'

ir1 = Range()
ir1.header = Header()
ir1.header.frame_id = 'ir_link_1'

ir2 = Range()
ir2.header = Header()
ir2.header.frame_id = 'ir_link_2'

ir3 = Range()
ir3.header = Header()
ir3.header.frame_id = 'ir_link_3'

ir4 = Range()
ir4.header = Header()
ir4.header.frame_id = 'ir_link_4'

ir5 = Range()
ir5.header = Header()
ir5.header.frame_id = 'ir_link_5'

ir6 = Range()
ir6.header = Header()
ir6.header.frame_id = 'ir_link_6'

ir7 = Range()
ir7.header = Header()
ir7.header.frame_id = 'ir_link_7'

ir8 = Range()
ir8.header = Header()
ir8.header.frame_id = 'ir_link_8'
'''

def callback_teleop(msg):

    global L, R, vel_motor_right, vel_motor_left

    lineal_x = msg.linear.x
    angular_z = msg.angular.z
    vel_motor_right = (2 * lineal_x + angular_z * L) / (2 * R)
    vel_motor_left = (2 * lineal_x - angular_z * L) / (2 * R)


def callback_sensor(msg):

    global sensor_data

    print(msg.data)


def serial_communication():

    rospy.init_node("serial_communication")

    # Subscribe to linear velocity x and angular velocity z from main computer or PS4 controller
    sub_teleop = rospy.Subscriber("/cmd_vel", Twist, callback_teleop)
    # Publish array with the two servo equivalents to the linear and angular velocities
    pub_teleop = rospy.Publisher("/servo_vel", Float64MultiArray, queue_size = 10)

    # Susbcribe to array with the 18 measurements (8 ultrasonics, 8 infrared, 2 encoders)
    sub_sensor = rospy.Subscriber("/sensor_data", Float64MultiArray, callback_sensor)
    # Publish the array of 18 values in 18 different topics
    pub_sensor = rospy.Publisher('/pub_data', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel_des.data[0] = vel_motor_left
        vel_des.data[1] = vel_motor_right
        pub_teleop.publish(vel_des)
    #   print('motor_izquierdo: {} \nmotor_derecho: {}'.format(vel_des.data[0], vel_des.data[1]))
    #   rospy.loginfo(vel_des.data)
        rate.sleep()


if __name__ == "__main__":

    try:
        serial_communication()

    except rospy.ROSInterruptException:
        pass
