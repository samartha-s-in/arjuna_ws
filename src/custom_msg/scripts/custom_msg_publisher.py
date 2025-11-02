#! /usr/bin/env python
                                                                                                                                               
import rospy
from custom_msg.msg import camera_data
from time import sleep

def raja():
    msg = camera_data()
    rate = rospy.Rate(10)
    while(True):
        msg.dog_length = 10.2
        msg.dog_height = 20.2
        msg.dog_distance = 25.1

        rospy.loginfo(msg)
        custom_data_pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
        
    rospy.init_node("custom_camera_data_publisher")

    custom_data_pub = rospy.Publisher("custom_camera_data" , camera_data , queue_size = 10)

    sleep(1)

    raja()