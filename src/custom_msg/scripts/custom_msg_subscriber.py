#! /usr/bin/env python
                                                                                                                                               
import rospy
from custom_msg.msg import camera_data
from time import sleep

length = 0
height = 0
distance = 0


def callback(data):
        global length , height , distance

        length = data.dog_length
        height = data.dog_height
        distance = data.dog_distance

def raja():
        global length , height , distance
        rate = rospy.Rate(10)
        while(True):
                print(length ,height , distance)
                rate.sleep()

if __name__ == "__main__":
    rospy.init_node("custom_camera_data_subscriber")
    sub = rospy.Subscriber('custom_camera_data',camera_data,callback)

    sleep(1)

    raja()