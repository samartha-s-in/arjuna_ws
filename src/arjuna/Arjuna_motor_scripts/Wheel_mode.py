#! /usr/bin/env python3

import rospy
from time import sleep
from STservo_sdk import * 

BAUDRATE     = 115200           
DEVICENAME   = '/dev/ttyUSB1'   

portHandler = PortHandler(DEVICENAME)

packetHandler = sts(portHandler)
    
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def wheel_mode(Motor_ID):
    Result, Error = packetHandler.WheelMode(Motor_ID)

def Run_Motor(Motor_ID , Motor_Speed , Motor_Accel):
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)

def main():
    wheel_mode(1)
    rate =rospy.Rate(10)
    while True:       
        Run_Motor(1 , 2000 , 0)
        sleep(5)
        Run_Motor(1 , 0 , 0) 
        sleep(5)
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("Motor_control" , anonymous = True)

    sleep(1)
    
    main()

    portHandler.closePort()
