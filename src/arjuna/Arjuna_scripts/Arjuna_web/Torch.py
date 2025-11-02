#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import smbus2
from time import sleep
 
bus = smbus2.SMBus(1)   # I2C channel (usually 1 on Jetson Nano)
arduino_address = 0x08  # Arduino Nano I2C address
Torch_data = 2

def callback(msg):
    global Torch_data
    Torch_data = msg.data


def Process():
    while True:
        global Torch_data
        bus.write_byte(arduino_address, Torch_data)
        print("Data sent to Arduino :" +str(Torch_data))
        sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("Torch_control" , anonymous = True)
    Torch_sub = rospy.Subscriber("Torch" , Int8 , callback)

    sleep(2)

    Process()



 
