#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf import transformations
import math
from time import sleep

quaternion = 0

def imu_callback(data):
    global quaternion
    quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
    
def NRB():
    global quaternion
    while True:
        euler = transformations.euler_from_quaternion(quaternion)
        yaw_ = round(math.degrees(euler[2]) , 2)
        print("Yaw in degrees : " + str(yaw_))

def main():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imu_callback)


if __name__ == '__main__':
    main()

    sleep(1)

    NRB()
