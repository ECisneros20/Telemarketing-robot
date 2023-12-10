#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

class PubBorrar:

    def __init__(self):

        rospy.init_node("pub_borrar", anonymous = False)


    def publisherFunctions(self):

        pub_borrar = rospy.Publisher("/sensor_data", Float32MultiArray, queue_size = 10)

        self.array = Float32MultiArray(data = 
                     [0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      0.20,
                      50,
                      50,
                      0])

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub_borrar.publish(self.array)
            rospy.loginfo("Executing!")
            self.rate.sleep()


if __name__ == "__main__":

    try:
        # Node initialization
        com = PubBorrar()
        rate = rospy.Rate(10)
        com.publisherFunctions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
