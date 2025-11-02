#! /usr/bin/env python
                                                                                                                                     
import rospy
from std_msgs.msg import Float64
from time import sleep

cpu_pub = rospy.Publisher("/CPU" , Float64 , queue_size = 10)
ram_pub = rospy.Publisher("/RAM" , Float64 , queue_size = 10)
temp_pub = rospy.Publisher("/TEMP" , Float64 , queue_size = 10)
battery_pub = rospy.Publisher("/BATTERY_PERCENTAGE" , Float64 , queue_size = 10)
left_ticks_pub = rospy.Publisher("/LEFT_TICKS" , Float64 , queue_size = 10)
right_ticks_pub = rospy.Publisher("/RIGHT_TICKS" , Float64 , queue_size = 10)

rospy.init_node("CPU_RAM_PUB" , anonymous = True)
rate = rospy.Rate(10)
while(True):
    cpu = 5.9
    ram = 99.9
    temp = 28.0
    battery_percentage = 90
    left_ticks = 12345.0
    right_ticks = 54321.0

    print(cpu,ram , temp , battery_percentage , left_ticks , right_ticks)
    cpu_pub.publish(cpu)
    ram_pub.publish(ram)
    temp_pub.publish(temp)
    battery_pub.publish(battery_percentage)
    left_ticks_pub.publish(left_ticks)
    right_ticks_pub.publish(right_ticks)

    sleep(1)

    cpu = 25.9
    ram = 299.9
    temp = 28.0
    battery_percentage = 80
    left_ticks = 78769.0
    right_ticks = 87222.0

    print(cpu,ram , temp , battery_percentage , left_ticks , right_ticks)
    cpu_pub.publish(cpu)
    ram_pub.publish(ram)
    temp_pub.publish(temp)
    battery_pub.publish(battery_percentage)
    left_ticks_pub.publish(left_ticks)
    right_ticks_pub.publish(right_ticks)

    sleep(3)

    rate.sleep()