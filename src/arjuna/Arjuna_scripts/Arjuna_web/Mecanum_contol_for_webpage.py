#! /usr/bin/env python3

'''
Company  Name: NEWRRO TECH
Date         : 7/6/2024
ROS Version  : Melodic
Website      : www.newrro.in
e-mail       : info@newrro.in
Contacts     : 8660875098 , 8217629665 , 8722278769
Social media : @newrro_tech , @last_bench_robots
This programs helps us to publish the total ticks travelled by 2 motors on "left_ticks" and "right_ticks" topics, and also subscribes for a topic called "cmd_vel" in-order to receive the velocity commands publihsed by any publisher.
'''

import rospy
from time import sleep
from STservo_sdk import * 
from std_msgs.msg import Int64 , Int8
from geometry_msgs.msg import Twist        
from time import sleep     

Left_velocity     = 0
Right_velocity    = 0

mecanum_control   = 0

BAUDRATE          = 115200          
DEVICENAME        = '/dev/ttyUSB1'   
docking_speed     = 4000
MOTOR_ACCL        = 0


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

def odom_callback(msg1):
    global Right_velocity , Left_velocity
    linear_vel = msg1.linear.x
    angular_vel = msg1.angular.z
    wheel_sep = 0.30

    Right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
    Left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)

    Right_velocity = int(Right_velocity * 10000)
    Left_velocity  = int(Left_velocity * 10000)

def mecanum_ctrl_callback(msg2):
    global mecanum_control
    mecanum_control = msg2.data 

def turn_left_dock():
    Run_Motor(1 , docking_speed , MOTOR_ACCL) 
    Run_Motor(3 , -docking_speed , MOTOR_ACCL)

    Run_Motor(2 , docking_speed , MOTOR_ACCL)
    Run_Motor(4 , -docking_speed , MOTOR_ACCL)

def turn_right_dock():
    Run_Motor(1 , -docking_speed , MOTOR_ACCL)
    Run_Motor(3 , docking_speed , MOTOR_ACCL) 

    Run_Motor(2 , -docking_speed , MOTOR_ACCL)
    Run_Motor(4 , docking_speed , MOTOR_ACCL)

def move_straight_dock():
    Run_Motor(1 , -docking_speed , MOTOR_ACCL)
    Run_Motor(4 , -docking_speed , MOTOR_ACCL)

    Run_Motor(2 , docking_speed , MOTOR_ACCL) 
    Run_Motor(3 , docking_speed , MOTOR_ACCL) 

def stop_dock():
    Run_Motor(1 , 0 , MOTOR_ACCL)
    Run_Motor(4 , 0 , MOTOR_ACCL)

    Run_Motor(2 , 0 , MOTOR_ACCL) 
    Run_Motor(3 , 0 , MOTOR_ACCL) 

def wheel_mode(Motor_ID):
    Result, Error = packetHandler.WheelMode(Motor_ID)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    elif Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))

def Run_Motor(Motor_ID , Motor_Speed , Motor_Accel):
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)
    if Result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(Result))
    if Error != 0:
        print("%s" % packetHandler.getRxPacketError(Error))


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
    global total_left_ticks , total_right_ticks , mecanum_control
    ENABLE_WHEEL_MODE()

    rate =rospy.Rate(10)
    while True:       
        if mecanum_control == 0:
            Run_Robot()
        elif mecanum_control == 1:
            turn_left_dock()
        elif mecanum_control == -1:
            turn_right_dock()

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Mecanum_control_for_webpage" , anonymous = True)
    cmd_vel_sub      = rospy.Subscriber("cmd_vel" , Twist , odom_callback) 
    mecanum_control_sub      = rospy.Subscriber("mecanum_control" , Int8 , mecanum_ctrl_callback) 

    print("")
    print("Publishers   : None ")
    print("Subscribers  : cmd_vel , mecanum_control")
    print("")

    sleep(2)
    
    main()

    portHandler.closePort()

