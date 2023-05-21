#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TwistToMotors():

    def __init__(self):

        rospy.init_node("twist_to_motors")
        # nodename = rospy.get_name()
        # rospy.loginfo("%s started" % nodename)

        self.w = rospy.get_param("~base_width", 0.3219)
        self.r = rospy.get_param("~radius", 0.1016)

        self.pub_lmotor = rospy.Publisher("/lwheel_vtarget", Float32, queue_size = 10)
        self.pub_rmotor = rospy.Publisher("/rwheel_vtarget", Float32, queue_size = 10)
        rospy.Subscriber("/cmd_vel", Twist, self.twistCallback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0

    def spin(self):

        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def spinOnce(self):

        # dx = (l + r) / 2
        # dr = (r - l) / w

        self.right = (2 * self.dx + self.dr * self.w) / (2 * self.r)
        self.left = (2 * self.dx - self.dr * self.w) / (2 * self.r)
        rospy.loginfo("motors: (%f, %f)", self.left, self.right)

        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)

        self.ticks_since_target += 1

    def twistCallback(self, msg):

        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        rospy.loginfo("particle: (%f, %f)", self.dx, self.dr)


if __name__ == "__main__":
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
