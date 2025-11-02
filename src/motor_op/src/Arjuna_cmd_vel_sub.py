#! /usr/bin/env python3

import rospy
from time import sleep
from STservo_sdk import * 
from geometry_msgs.msg import Twist        
from time import sleep     

Left_velocity     = 0
Right_velocity    = 0

BAUDRATE          = 115200          
DEVICENAME        = '/dev/motor'   
MOTOR_ACCL        = 0

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

def odom_callback(msg1):
    global Right_velocity , Left_velocity
    linear_vel = msg1.linear.x
    angular_vel = msg1.angular.z
    wheel_sep = 0.30

    Right_velocity =((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
    Left_velocity =((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)

    Right_velocity = int(Right_velocity * 10000)
    Left_velocity  = int(Left_velocity * 10000)

def wheel_mode(Motor_ID):
    Result, Error = packetHandler.WheelMode(Motor_ID)

def Run_Motor(Motor_ID , Motor_Speed , Motor_Accel):
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, 
Motor_Accel)

def ENABLE_WHEEL_MODE():
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def Run_Robot():
    global MOTOR_ACCL , Left_velocity , Right_velocity

    Run_Motor(1 , -Left_velocity , MOTOR_ACCL) #Left_motor
    Run_Motor(4 , -Left_velocity , MOTOR_ACCL) #Left_motor

    Run_Motor(2 , Right_velocity , MOTOR_ACCL) #Right_motor 
    Run_Motor(3 , Right_velocity , MOTOR_ACCL) #Right_motor 

def main():

    ENABLE_WHEEL_MODE()

    rate =rospy.Rate(10)
    while True:       
        print("Subscriber for cmd_vel")
        Run_Robot()
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Arjuna_cmd_vel_sub" , anonymous = True)

    cmd_vel_sub    = rospy.Subscriber("cmd_vel" , Twist , odom_callback) 
    
    print("")
    print("Publishers   : None")
    print("Subscribers  : cmd_vel")
    print("")

    sleep(2)
    
    main()

    portHandler.closePort()
