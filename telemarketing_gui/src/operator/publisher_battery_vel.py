#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Float32, Int32

def talker():
    battery_level = 42
    speed = 0.5
    pub_battery = rospy.Publisher('/battery_level', Int32, queue_size=10) # Float32
    pub_speed = rospy.Publisher('/speed', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        #battery_level = battery_level+0.01
        if battery_level<90:
            battery_level = battery_level+1
        speed = speed+0.01
        #battery_level = round(battery_level,2)
        #speed = round(speed,2)
        rospy.loginfo(battery_level)
        rospy.loginfo(speed)
        pub_battery.publish(battery_level)
        pub_speed.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass