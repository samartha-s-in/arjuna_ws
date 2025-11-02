#!/usr/bin/env python3
import rospy
import actionlib
from action_example.msg import CountAction, CountGoal

if __name__ == '__main__':
    rospy.init_node('count_action_client')
    client = actionlib.SimpleActionClient('count', CountAction)

    goal = CountGoal()
    goal.target = 5  # Set the target count
    client.send_goal(goal)

    client.wait_for_server()
    while not client.wait_for_result(rospy.Duration(1.0)):
        print("Waiting for result...")

    #Above loop repeatedly checks whether the action server has completed its task for every one second.
    #If the server hasn’t returned a result yet, it prints the above statement.

    result = client.get_result()
    rospy.loginfo("Result = %s" %(result.total))
