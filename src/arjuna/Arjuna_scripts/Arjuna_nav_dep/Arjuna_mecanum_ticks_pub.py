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

selector_r  = 0
selector_l  = 0

Left_ticks  = 0
Right_ticks = 0

Left_velocity     = 0
Right_velocity    = 0

prev_left_ticks   = 0
prev_right_ticks  = 0

total_left_ticks  = 0
total_right_ticks = 0

encoder_maximum   = 32768
mecanum_control   = 0

BAUDRATE          = 115200          
DEVICENAME        = '/dev/motor'   
docking_speed     = 400
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

def present_pos(Motor_ID):
    Position, Speed, Result, Error = packetHandler.ReadPosSpeed(Motor_ID)
    if Result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(Result))
    if Error != 0:
        print(packetHandler.getRxPacketError(Error))
    return Position

def ENABLE_WHEEL_MODE():
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def right_tick_count():
    global Right_ticks , Right_velocity , total_right_ticks ,prev_right_ticks , selector_r

    Right_ticks = int(present_pos(2)/3.5)

    if Right_velocity > 0:
        selector_r = 1
    elif Right_velocity < 0:
        selector_r = 2

    if selector_r == 1 and prev_right_ticks < Right_ticks:
        if total_right_ticks >= encoder_maximum:
            total_right_ticks = 0
        else:
            total_right_ticks += abs(Right_ticks - prev_right_ticks)
    elif selector_r == 2 and prev_right_ticks > Right_ticks:
        if total_right_ticks <= 0:
            total_right_ticks = encoder_maximum
        else:
            total_right_ticks += (Right_ticks - prev_right_ticks)

    prev_right_ticks = Right_ticks

    Right_ticks = total_right_ticks
    right_ticks_pub.publish(Right_ticks)


def left_tick_count():
    global Left_velocity, Left_ticks , total_left_ticks , prev_left_ticks , selector_l

    Left_ticks = int(present_pos(1)/3.5)

    if Left_velocity > 0:
        selector_l = 1
    elif Left_velocity < 0:
        selector_l = 2

    if selector_l == 1 and prev_left_ticks > Left_ticks:
        if total_left_ticks >= encoder_maximum:
            total_left_ticks = 0
        else:
            total_left_ticks += abs(Left_ticks - prev_left_ticks)
    elif selector_l == 2 and prev_left_ticks < Left_ticks:
        if total_left_ticks <= 0:
            total_left_ticks = encoder_maximum
        else:
            total_left_ticks -= (Left_ticks - prev_left_ticks)
    
    prev_left_ticks = Left_ticks

    Left_ticks = total_left_ticks
    left_ticks_pub.publish(Left_ticks)    

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
        left_tick_count()
        right_tick_count()
        print("Left_Ticks : " + str(total_left_ticks) + " | " + "Right_Ticks : " + str(total_right_ticks))
        if mecanum_control == 0:
            Run_Robot()
        elif mecanum_control == 1:
            turn_left_dock()
        elif mecanum_control == -1:
            turn_right_dock()
        elif mecanum_control == 2:
            move_straight_dock()
        elif mecanum_control == 3:
            stop_dock()

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Arjuna_Ticks_Publisher" , anonymous = True)

    left_ticks_pub   = rospy.Publisher("left_ticks" , Int64, queue_size = 10)
    right_ticks_pub  = rospy.Publisher("right_ticks" , Int64, queue_size = 10)
    cmd_vel_sub      = rospy.Subscriber("cmd_vel" , Twist , odom_callback) 
    mecanum_control_sub      = rospy.Subscriber("mecanum_control" , Int8 , mecanum_ctrl_callback) 

    print("")
    print("Publishers   : left_ticks , right_ticks")
    print("Subscribers  : cmd_vel")
    print("")

    sleep(2)
    
    main()

    portHandler.closePort()

