#!/usr/bin/env python
# Link: https://www.youtube.com/watch?v=aE7RQNhwnPQ / 7:45

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import LaserScan


''' Teleop '''

R = 0.1016 # Wheel radius (m)
L = 0.3219 # Distance between the wheels (m)
vel_motor_right = 0.0 # Angular velocity for right motor
vel_motor_left = 0.0 # Angular velocity for left motor

vel_des = Float64MultiArray()
vel_des.data.append(0.0)
vel_des.data.append(0.0)


''' Data sensor subscription '''

num_sensors = 18
sensor_data = Float64MultiArray()

for i in range(num_sensors):
    sensor_data.data.append(0.0)


''' Data sensor publication '''

us_msg_1 = LaserScan()
us_msg_1.header = Header()
us_msg_1.header.frame_id = 'us_link_1'

us_msg_2 = LaserScan()
us_msg_2.header = Header()
us_msg_2.header.frame_id = 'us_link_2'

us_msg_3 = LaserScan()
us_msg_3.header = Header()
us_msg_3.header.frame_id = 'us_link_3'

us_msg_4 = LaserScan()
us_msg_4.header = Header()
us_msg_4.header.frame_id = 'us_link_4'

us_msg_5 = LaserScan()
us_msg_5.header = Header()
us_msg_5.header.frame_id = 'us_link_5'

us_msg_6 = LaserScan()
us_msg_6.header = Header()
us_msg_6.header.frame_id = 'us_link_6'

us_msg_7 = LaserScan()
us_msg_7.header = Header()
us_msg_7.header.frame_id = 'us_link_7'

us_msg_8 = LaserScan()
us_msg_8.header = Header()
us_msg_8.header.frame_id = 'us_link_8'

ir_msg_1 = LaserScan()
ir_msg_1.header = Header()
ir_msg_1.header.frame_id = 'ir_link_1'

ir_msg_2 = LaserScan()
ir_msg_2.header = Header()
ir_msg_2.header.frame_id = 'ir_link_2'

ir_msg_3 = LaserScan()
ir_msg_3.header = Header()
ir_msg_3.header.frame_id = 'ir_link_3'

ir_msg_4 = LaserScan()
ir_msg_4.header = Header()
ir_msg_4.header.frame_id = 'ir_link_4'

ir_msg_5 = LaserScan()
ir_msg_5.header = Header()
ir_msg_5.header.frame_id = 'ir_link_5'

ir_msg_6 = LaserScan()
ir_msg_6.header = Header()
ir_msg_6.header.frame_id = 'ir_link_6'

ir_msg_7 = LaserScan()
ir_msg_7.header = Header()
ir_msg_7.header.frame_id = 'ir_link_7'

ir_msg_8 = LaserScan()
ir_msg_8.header = Header()
ir_msg_8.header.frame_id = 'ir_link_8'


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

    rospy.init_node('serial_communication')

    # Recepcion de velocidad lineal y angular desde el mando de PS4
    sub_teleop = rospy.Subscriber('/cmd_vel', Twist, callback_teleop)
    # Envio de la velocidad angular de los dos motores hacia el microcontrolador
    pub_teleop = rospy.Publisher('/vel_motor', Float64MultiArray, queue_size=10)

    # Recepcion de los datos de los 16 sensores desde el microcontrolador
    sub_sensor = rospy.Subscriber('/get_data', Float64MultiArray, callback_sensor)
    # Envio de los datos de los sensores para lectura en RVIz
    pub_sensor = rospy.Publisher('/pub_data', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel_des.data[0] = vel_motor_left
        vel_des.data[1] = vel_motor_right
        pub_teleop.publish(vel_des)
    #   print('motor_izquierdo: {} \nmotor_derecho: {}'.format(vel_des.data[0], vel_des.data[1]))
    #   rospy.loginfo(vel_des.data)
        rate.sleep()


if __name__ == '__main__':
    try:
        serial_communication()
    except rospy.ROSInterruptException:
        pass
