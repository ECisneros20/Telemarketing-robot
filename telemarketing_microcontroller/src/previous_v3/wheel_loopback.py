#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

class WheelLoopback():

    def __init__(self):

        rospy.init_node("wheel_loopback")
        # self.nodename = rospy.get_name()
        # rospy.loginfo("%s started" % self.nodename)

        self.rate = rospy.get_param("~rate", 200)
        self.timeout_secs = rospy.get_param("~timeout_secs", 0.5)
        self.ticks_meter = float(rospy.get_param("~ticks_meter", 50))
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 255))
        self.latest_motor = 0

        self.wheel = 0

        rospy.Subscriber("/motor", Int16, self.motor_callback)

        self.pub_wheel = rospy.Publisher("/wheel", Int16, queue_size = 10)

    def spin(self):

        r = rospy.Rate
        self.secs_since_target = self.timeout_secs
        self.then = rospy.Time.now()
        self.latest_msg_time = rospy.Time.now()
        rospy.loginfo("-D- spinning")

        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.secs_since_target < self.timeout_secs:
                self.spinOnce()
                r.sleep
                self.secs_since_target = rospy.Time.now() - self.latest_msg_time
                self.secs_since_target = self.secs_since_target.to_sec()
                #rospy.loginfo("  inside: secs_since_target: %0.3f" % self.secs_since_target)

            # it's been more than timeout_ticks since we recieved a message
            self.secs_since_target = rospy.Time.now() - self.latest_msg_time
            self.secs_since_target = self.secs_since_target.to_sec()
            # rospy.loginfo("  outside: secs_since_target: %0.3f" % self.secs_since_target)
            self.velocity = 0
            r.sleep

    def spinOnce(self):

        self.velocity = self.latest_motor / self.velocity_scale
        if abs(self.velocity) > 0:
            self.seconds_per_tick = abs( 1 / (self.velocity * self.ticks_meter))
            elapsed = rospy.Time.now() - self.then
            elapsed = elapsed.to_sec()
            rospy.loginfo("spinOnce: vel=%0.3f sec/tick=%0.3f elapsed:%0.3f" % (self.velocity, self.seconds_per_tick, elapsed))

            if (elapsed > self.seconds_per_tick):
                rospy.loginfo("incrementing wheel")
                if self.velocity > 0:
                    self.wheel += 1
                else:
                    self.wheel -= 1
                self.pub_wheel.publish(self.wheel)
                self.then = rospy.Time.now()

    def motor_callback(self, msg):

        self.latest_motor = msg.data
        self.latest_msg_time = rospy.Time.now()


if __name__ == "__main__":
    try:
        wheelLoopback = WheelLoopback()
        wheelLoopback.spin()
    except rospy.ROSInterruptException:
        pass
