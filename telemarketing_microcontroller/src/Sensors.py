#!/usr/bin/env python
# Link: https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/347

import rospy
from std_msgs.msg import Header, Float32MultiArray, Bool
from sensor_msgs.msg import Range
from math import pi

# Publisher (17)
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
# /emerg         -   Bool                -   to Main Computer                        -   Separate array of 19 measurements (1)
# /reset         -   Bool                -   to Microcontroller                 -   Separate array of 19 measurements (

# Subscriber (1)
# /sensor_data   -   Float32MultiArray   -   from Microcontroller                    -   Array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder signals, 1 bumper)

class SerialComSensors:

    def __init__(self):

        # Sensors measurement ranges
        self.FIELD_OF_VIEW_US = pi/6
        self.MIN_RANGE_US = 0.20
        self.MAX_RANGE_US = 5.00
        self.FIELD_OF_VIEW_IR = pi/12
        self.MIN_RANGE_IR = 0.00
        self.MAX_RANGE_IR = 0.35
        self.seq = 0
        self.reset = Bool(data = 1)

        rospy.init_node("serial_communication_sensors", anonymous = False)

        # Susbcribe to array with 19 measurements (8 ultrasonics, 8 infrared, 2 encoder signals, 1 bumper)
        self.sub_sensor = rospy.Subscriber("/sensor_data", Float32MultiArray, self.callback_sensor)
        self.sub_resetBumper = rospy.Subscriber("/reset_gui", Bool, self.callback_resetBumper)

        # Messages to publish
        self.sonar_1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido1_1"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido2_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido3_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido4_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido5_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido6_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido7_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.sonar_8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "ultrasonido8_change_frame"), radiation_type = 0, field_of_view = self.FIELD_OF_VIEW_US,
                         min_range = self.MIN_RANGE_US, max_range = self.MAX_RANGE_US, range = self.MIN_RANGE_US)
        self.ir_1 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo1_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_2 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo2_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_3 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo3_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_4 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo4_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_5 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo5_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_6 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo6_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_7 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo7_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.ir_8 = Range(header = Header(seq = self.seq, stamp = rospy.Time.now(), frame_id = "infrarrojo8_change_frame"), radiation_type = 1, field_of_view = self.FIELD_OF_VIEW_IR,
                         min_range = self.MIN_RANGE_IR, max_range = self.MAX_RANGE_IR, range = self.MIN_RANGE_IR)
        self.emerg = Bool(data = 0)

    def callback_resetBumper(self, msg):
    	self.reset = Bool(data = msg.data)

    def callback_sensor(self, msg):

        def get_range(range_msg, range_measurement):

            if range_measurement < range_msg.min_range:
                return float("-Inf")
            elif range_measurement > range_msg.max_range:
                return float("Inf")
            else:
                return range_measurement

        self.seq += 1

        # 8 sonar publishers
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
        #print("US: ", self.sonar_1.range, "-", self.sonar_2.range, "-", self.sonar_3.range, "-", self.sonar_4.range, "-", self.sonar_5.range, "-", self.sonar_6.range, "-", self.sonar_7.range, "-", self.sonar_8.range)
        #print("IR: ", self.ir_1.range, "-", self.ir_2.range, "-", self.ir_3.range, "-", self.ir_4.range, "-", self.ir_5.range, "-", self.ir_6.range, "-", self.ir_7.range, "-", self.ir_8.range)

        # Emergency publisher
        self.emerg.data = msg.data[18]


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
        pub_emerg = rospy.Publisher("/emerg", Bool, queue_size = 10)
        pub_reset = rospy.Publisher("/reset", Bool, queue_size = 10)

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
            pub_emerg.publish(self.emerg)
            pub_reset.publish(self.reset)
            rospy.loginfo("Executing!")
            self.rate.sleep()


if __name__ == "__main__":

    try:
        # Node initialization
        com = SerialComSensors()
        rate = rospy.Rate(10)
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
