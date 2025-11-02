#! /usr/bin/env python3

import rospy
from time import sleep
from STservo_sdk import * 

BAUDRATE     = 115200           
DEVICENAME   = '/dev/ttyUSB1' 

portHandler = PortHandler(DEVICENAME)

packetHandler = sts(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def wheel_mode(Motor_ID):
    Result, Error = packetHandler.WheelMode(Motor_ID)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    if Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))

def Run_Motor(Motor_ID , Motor_Speed , Motor_Accel):
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    if Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))
def present_pos(Motor_ID):
    Ticks, Speed, Result, Error = packetHandler.ReadPosSpeed(Motor_ID)
    if Result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(Result))
    if Error != 0:
        print(packetHandler.getRxPacketError(Error))
    return Ticks

def main():
    Motor_1_Ticks = 0
    wheel_mode(1)

    rate =rospy.Rate(10)
    while True:       
        Motor_1_Ticks = present_pos(1)
        Run_Motor(1 , 500 , 0) #Left_motor
        print(Motor_1_Ticks)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Motor_control" , anonymous = True)

    sleep(1)
    
    main()

portHandler.closePort()

