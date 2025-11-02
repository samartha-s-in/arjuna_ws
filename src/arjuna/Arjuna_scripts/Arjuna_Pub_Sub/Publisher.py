#! /usr/bin/env python
                                                                                                                                               
import rospy
from std_msgs.msg import Float64


def Publisher():
    rate = rospy.Rate(10)
    while(True):
        value = 19.0
        rospy.loginfo(value)
        Pub.publish(value)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Simple_Publisher" , anonymous = True)
    Pub = rospy.Publisher("distance" , Float64 , queue_size = 10)

    Publisher()