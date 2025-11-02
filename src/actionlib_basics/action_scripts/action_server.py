#!/usr/bin/env python

import rospy
import actionlib
from action_example.msg import CountAction, CountFeedback, CountResult

def execute(goal):
    feedback = CountFeedback()
    total = 0

    for i in range(goal.target):
        total += 1
        feedback.progress = total
        print(total)
        server.publish_feedback(feedback)
        rospy.sleep(1)  

    result = CountResult()
    result.total = total
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('count_action_server')
    server = actionlib.SimpleActionServer('count', CountAction, execute, False)
    server.start()    
    rospy.spin()


