#! /usr/bin/env python

import sys
import rospy
from custom_msg.srv import *

def Response_from_server(A, B):
    Client_init = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    Request = Client_init(A, B)
    rospy.wait_for_service('add_two_ints')
    print("Response from Server : %s" %(Request) )

if __name__ == "__main__":
    A = int(input("Enter the 1st number : "))
    B = int(input("Enter the 2nd number : "))
    print("Requesting Server : %s + %s = ?"%(A, B))
    Response_from_server(A, B)
