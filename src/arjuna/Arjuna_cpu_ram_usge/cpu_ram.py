#! /usr/bin/env python
import rospy
import psutil
from std_msgs.msg import Float64
from time import sleep

cpu_pub = None
ram_pub = None

def publish_paras():
    global cpu_pub , ram_pub 
    rate = rospy.Rate(1)
    while True:
        cpu_used = psutil.cpu_percent(1)
        ram_used =  psutil.virtual_memory()[2]
        print("CPU used : " + str(cpu_used) + " and " + "RAM used : " + str(ram_used))
        cpu_pub.publish(cpu_used)
        ram_pub.publish(ram_used)
        rate.sleep()

def main():
    global cpu_pub , ram_pub , battery_pub
    rospy.init_node("CPU_RAM_PUB" , anonymous = True)

    cpu_pub = rospy.Publisher("/CPU" , Float64 , queue_size = 10)
    ram_pub = rospy.Publisher("/RAM" , Float64 , queue_size = 10)

main()
publish_paras()