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
from geometry_msgs.msg import Twist
import math

# Navigation parameters
YAW_PRECISION = math.pi / 20  # +/- ~9 degrees precision for orientation
DIST_PRECISION = 0.05        # Goal point radius in meters (5cm)
LINEAR_VELOCITY = 0.5      # Linear velocity of the robot in m/s
ANGULAR_VELOCITY = 0.8      # Angular velocity of the robot in rad/s

# States
# 0 = Rotating to goal orientation
# 1 = Moving straight
# 2 = Goal reached
# 3 = Obstacle handling
state_ = 0

cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

########################## UTILITY FUNCTIONS ########################################

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi"""
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle
    
########################## NAVIGATION FUNCTIONS ########################################

def fix_yaw(des_pos, yaw_, position_, state_):
    """Align robot toward goal position"""
    global YAW_PRECISION
    
    # Calculate desired yaw angle to goal
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    # Create and publish twist message
    twist_msg = Twist()
    
    # Calculate appropriate angular velocity based on error
    if math.fabs(err_yaw) > YAW_PRECISION:
        # Use proportional control for smoother rotation
        k_p = 1.0  # Proportional gain
        angular_z = k_p * err_yaw
        
        # Clamp to max velocity
        if angular_z > ANGULAR_VELOCITY:
            angular_z = ANGULAR_VELOCITY
        elif angular_z < -ANGULAR_VELOCITY:
            angular_z = -ANGULAR_VELOCITY
            
        twist_msg.angular.z = angular_z
        print("Aligning: Current yaw={:.2f}, Desired={:.2f}, Error={:.2f}".format(
            yaw_, desired_yaw, err_yaw))
    else:
        print("Yaw aligned with goal")
        # Only change state if alignment is complete
        state_ = 1
        print("Robot State: " + str(state_))
    
    cmd_pub.publish(twist_msg)
    return yaw_, position_, state_

def go_straight_ahead(des_pos, yaw_, position_, state_):
    """Move straight toward goal while maintaining orientation"""
    global YAW_PRECISION, DIST_PRECISION
    
    # Calculate desired yaw and position error
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    # Check if we've reached the goal
    if err_pos <= DIST_PRECISION:
        print("Goal reached!")
        state_ = 2
        print("Robot State: " + str(state_))
        return yaw_, position_, state_
    
    # If orientation error is too large, go back to alignment state
    if math.fabs(err_yaw) > YAW_PRECISION:
        print("Orientation error too large, realigning")
        state_ = 0
        print("Robot State: " + str(state_))
        return yaw_, position_, state_
    
    # Otherwise, keep moving forward with small orientation corrections
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_VELOCITY
    
    # Apply minor correction to orientation while moving
    if err_yaw != 0:
        # Small proportional control for minor adjustments while moving
        k_p = 0.5
        twist_msg.angular.z = k_p * err_yaw
    
    cmd_pub.publish(twist_msg)
    
    return yaw_, position_, state_

########################## MOVEMENT FUNCTIONS ########################################

def stop():
    """Stop the robot"""
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)

def move_straight():
    """Move robot straight ahead"""
    global LINEAR_VELOCITY
    
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_VELOCITY
    cmd_pub.publish(twist_msg)

def turn_left():
    """Turn robot left"""
    global ANGULAR_VELOCITY
    
    twist_msg = Twist()
    twist_msg.angular.z = ANGULAR_VELOCITY
    cmd_pub.publish(twist_msg)

def turn_right():
    """Turn robot right"""
    global ANGULAR_VELOCITY
    
    twist_msg = Twist()
    twist_msg.angular.z = -ANGULAR_VELOCITY
    cmd_pub.publish(twist_msg)
