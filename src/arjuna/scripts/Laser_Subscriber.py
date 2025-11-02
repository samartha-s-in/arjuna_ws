#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    global regions
    regions = {
        'front_L' : min(min(msg.ranges[0:130]), 10), 
        'fleft'   : min(min(msg.ranges[131:230]), 10),  
        'left'    : min(min(msg.ranges[231:280]), 10),
        'right'   : min(min(msg.ranges[571:620]), 10), 
        'fright'  : min(min(msg.ranges[621:720]), 10), 
        'front_R' : min(min(msg.ranges[721:850]), 10)
    }
    front_obstacle = regions['front_L'] < 0.3 or regions['front_R'] < 0.3
    left_obstacle = regions['fleft'] < 0.3 or regions['left'] < 0.3
    right_obstacle = regions['fright'] < 0.3 or regions['right'] < 0.3

    if front_obstacle:
        rospy.loginfo("Front Obstacle")
    elif left_obstacle:
        rospy.loginfo("Left Obstacle")
    elif right_obstacle:
        rospy.loginfo("Right Obstacle")
    else:
        rospy.loginfo("No Obstacle")

if __name__ == "__main__":
    rospy.init_node('Simple_Subscriber')
    sub = rospy.Subscriber('/scan',LaserScan,clbk_laser)
    rospy.spin()
