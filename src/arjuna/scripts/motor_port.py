#! /usr/bin/env python3

import rospy
from time import sleep
from STservo_sdk import * 
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist        
from time import sleep

selector_r  = 0
selector_l  = 0

Left_ticks  = 0
Right_ticks = 0

L_velocity     = 0
R_velocity    = 0

prev_left_ticks   = 0
prev_right_ticks  = 0

total_left_ticks  = 0
total_right_ticks = 0

encoder_maximum = 32000
BAUDRATE        = 115200

DEVICENAME = '/dev/ttyUSB0'
portHandler = None
packetHandler = None

def devicename(device_name):
    """
    Sets the device name (port name) and initializes PortHandler & PacketHandler.
    Returns True if successful, False otherwise.
    """
    global DEVICENAME, portHandler, packetHandler

    portHandler   = PortHandler(DEVICENAME) 
    packetHandler = sts(portHandler)

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return False  # Failure
        quit()
	    

    return True  # Success

def get_devicename():
    """Returns the current device name or None if not set."""
    return DEVICENAME

devicename(DEVICENAME)

