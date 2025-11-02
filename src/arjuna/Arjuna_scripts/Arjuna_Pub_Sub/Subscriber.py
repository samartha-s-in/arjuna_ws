#! /usr/bin/env python
                                                                                                                                               
import rospy
from std_msgs.msg import Float64

def callback(msg):
    global distance
    distance = msg.data
    print(distance)

if __name__ == "__main__":
    rospy.init_node('Simple_Subscriber')
    sub = rospy.Subscriber('/scan',Float64,callback)
    rospy.spin()
