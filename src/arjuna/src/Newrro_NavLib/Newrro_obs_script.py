#! /usr/bin/env python

'''
Company Name : NEWRRO TECH
Date         : 7/6/2024
ROS Version  : Melodic
Website      : www.newrro.in
e-mail       : info@newrro.in
Contacts     : 8660875098 , 8217629665 , 8722278769
Social media : @newrro_tech , @last_bench_robots
'''

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Navigation parameters
LINEAR_VELOCITY = 0.5      # Forward speed (m/s)
ANGULAR_VELOCITY = 0.8      # Angular velocity of the robot in rad/s
OBSTACLE_DIST_THRESHOLD = 0.3  # Distance threshold for obstacle detection (30cm)

cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


########################## OBSTACLE AVOIDANCE FUNCTIONS ########################################

def is_path_clear(regions):
    """Check if path is clear of obstacles"""
    global OBSTACLE_DIST_THRESHOLD
    
    if not regions:
        return True  # No LIDAR data yet, assume clear
    
    # Consider path clear if all regions are beyond threshold
    return (regions['front_L'] > OBSTACLE_DIST_THRESHOLD and 
            regions['front_R'] > OBSTACLE_DIST_THRESHOLD and
            regions['fleft'] > OBSTACLE_DIST_THRESHOLD and
            regions['fright'] > OBSTACLE_DIST_THRESHOLD)

def avoid_obstacle(regions):

    """Generate avoidance command based on obstacle configuration"""
    global OBSTACLE_DIST_THRESHOLD, ANGULAR_VELOCITY
    
    twist_msg = Twist()
    
    # Check different obstacle configurations
    front_obstacle = regions['front_L'] < OBSTACLE_DIST_THRESHOLD or regions['front_R'] < OBSTACLE_DIST_THRESHOLD
    left_obstacle = regions['fleft'] < OBSTACLE_DIST_THRESHOLD or regions['left'] < OBSTACLE_DIST_THRESHOLD
    right_obstacle = regions['fright'] < OBSTACLE_DIST_THRESHOLD or regions['right'] < OBSTACLE_DIST_THRESHOLD
    
    # Decision tree for obstacle avoidance
    if front_obstacle and left_obstacle and right_obstacle:
        # Surrounded by obstacles - back up slightly
        print("Surrounded by obstacles - backing up")
        twist_msg.linear.x = -0.1
        twist_msg.angular.z = 0
    elif front_obstacle and left_obstacle:
        # Obstacles in front and left - turn right
        print("Obstacles front and left - turning right")
        twist_msg.linear.x = 0
        twist_msg.angular.z = -ANGULAR_VELOCITY
    elif front_obstacle and right_obstacle:
        # Obstacles in front and right - turn left
        print("Obstacles front and right - turning left")
        twist_msg.linear.x = 0
        twist_msg.angular.z = ANGULAR_VELOCITY
    elif front_obstacle:
        # Only front obstacle - check which side has more space
        if regions['left'] > regions['right']:
            print("Front obstacle - turning left (more space)")
            twist_msg.linear.x = 0
            twist_msg.angular.z = ANGULAR_VELOCITY
        else:
            print("Front obstacle - turning right (more space)")
            twist_msg.linear.x = 0
            twist_msg.angular.z = -ANGULAR_VELOCITY
    elif left_obstacle:
        # Left obstacle - slight right turn while moving
        print("Left obstacle - adjusting right")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = -ANGULAR_VELOCITY * 0.5
    elif right_obstacle:
        # Right obstacle - slight left turn while moving
        print("Right obstacle - adjusting left")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = ANGULAR_VELOCITY * 0.5
    else:
        # No obstacles detected - clear path
        return None
    
    return twist_msg

