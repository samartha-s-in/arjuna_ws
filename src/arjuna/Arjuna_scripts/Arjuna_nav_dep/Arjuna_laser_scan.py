#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    regions = [
         min(min(msg.ranges[0:130]), 10), #front_L
        ## min(min(msg.ranges[131:230]), 10), #Fleft
      #  min(min(msg.ranges[231:280]), 10), #Left#

        #min(min(msg.ranges[570:620]), 10), #Right
      #  min(min(msg.ranges[621:720]), 10),  #Fright
      #  min(min(msg.ranges[721:850]), 10), #front_R
    ]
    print(regions)
    
def main():
    rospy.init_node('reading_laser')
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()