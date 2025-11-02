#! /usr/bin/env python

import rospy    
from custom_msg.srv import AddTwoInts,AddTwoIntsResponse

def Respond_to_client(req):
    print("Request from Client : [%s + %s = ?]"%(req.a, req.b))
    print("Response to Client : [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    print("------------------------------------------------------------")
    return AddTwoIntsResponse(req.a + req.b)

def server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, Respond_to_client)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    server()