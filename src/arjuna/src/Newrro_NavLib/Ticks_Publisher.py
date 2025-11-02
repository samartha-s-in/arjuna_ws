#! /usr/bin/env python3

import rospy
from Newrro import*

Left_velocity     = 0
Right_velocity    = 0

Left_Motor_Ticks  = 0
Right_Motor_Ticks = 0

MOTOR_ACCL        = 0

def odom_callback(msg1):
    global Right_velocity , Left_velocity
    linear_vel = msg1.linear.x
    angular_vel = msg1.angular.z
    wheel_sep = 0.30

    Right_velocity =((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
    Left_velocity =((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)

    Right_velocity = int(Right_velocity * 10000)
    Left_velocity  = int(Left_velocity * 10000)

def Run_Robot():
    global MOTOR_ACCL , Left_velocity , Right_velocity

    Run_Motor(1 , -Left_velocity , MOTOR_ACCL) #Left_motor
    Run_Motor(4 , -Left_velocity , MOTOR_ACCL) #Left_motor

    Run_Motor(2 , Right_velocity , MOTOR_ACCL) #Right_motor 
    Run_Motor(3 , Right_velocity , MOTOR_ACCL) #Right_motor 

def main():
    global total_left_ticks , total_right_ticks

    rospy.init_node("Arjuna_Ticks_Publisher" , anonymous = True)

    left_ticks_pub =rospy.Publisher("left_ticks" ,Int64, queue_size = 10)

    right_ticks_pub=rospy.Publisher("right_ticks" ,Int64, queue_size = 10)

    cmd_vel_sub    = rospy.Subscriber("cmd_vel" , Twist , odom_callback)

    print("")
    print("Publishers   : left_ticks , right_ticks")
    print("Subscribers  : cmd_vel")
    print("")

    rate =rospy.Rate(10)
    ENABLE_WHEEL_MODE()
    while True:       
        Left_Motor_Ticks  = left_tick_count(Left_velocity)
        Right_Motor_Ticks = right_tick_count(Right_velocity)
        left_ticks_pub.publish(Left_Motor_Ticks)  
        right_ticks_pub.publish(Right_Motor_Ticks)
        print("Left_Ticks : " + str(Left_Motor_Ticks) + " | " + "Right_Ticks : " + str(Right_Motor_Ticks))
        Run_Robot()
        rate.sleep()

    portHandler.closePort()


main()

