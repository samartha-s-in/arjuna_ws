#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
import math
from time import sleep

# Import functions from Newrro_Navigation
from Newrro_NavLib.Newrro_Navigation import *

# Import functions from Newrro_obs_script
from Newrro_NavLib.Newrro_obs_script import *

# Navigation parameters
LINEAR_VELOCITY = 0.5      # Linear velocity of the robot in m/s
ANGULAR_VELOCITY = 0.8      # Angular velocity of the robot in rad/s
OBSTACLE_DIST_THRESHOLD = 0.3  # Distance threshold for obstacle detection (30cm)
RECOVERY_TIMEOUT = 5.0      # Time threshold for stuck detection (seconds)

# Global variables
position_ = Point()          # Current robot position
yaw_ = 0                     # Current robot orientation
regions = {}                 # LIDAR regions
state_ = 0                   # Robot state: 0=rotate, 1=move straight, 2=goal reached
last_position_ = Point()     # Last position for stuck detection
stuck_time_ = 0              # Time counter for stuck detection
cmd_pub = None               # Publisher for velocity commands

########################## CALLBACK FUNCTIONS ########################################

def clbk_laser(msg):
    """Process LIDAR data into regions"""
    global regions
    
    regions = {
        'front_L': min(min(msg.ranges[0:130]), 10),
        'fleft': min(min(msg.ranges[131:230]), 10),
        'left': min(min(msg.ranges[231:280]), 10),
        'right': min(min(msg.ranges[571:620]), 10),
        'fright': min(min(msg.ranges[621:720]), 10),
        'front_R': min(min(msg.ranges[721:850]), 10)
    }

def clbk_odom(msg):
    """Process odometry data"""
    global position_, yaw_
    
    # Update position
    position_ = msg.pose.pose.position
    
    # Extract yaw from quaternion
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

########################## MAIN FUNCTIONS ########################################

def check_if_stuck():
    """Check if robot is stuck and return a recovery behavior if needed"""
    global position_, last_position_, stuck_time_, RECOVERY_TIMEOUT, state_
    
    # Calculate distance moved since last check
    dist_moved = math.sqrt(pow(position_.x - last_position_.x, 2) + 
                          pow(position_.y - last_position_.y, 2))
    
    # If barely moving and in movement state
    if dist_moved < 0.01 and state_ == 1:
        stuck_time_ += 0.1  # Assuming 10Hz rate
        if stuck_time_ > RECOVERY_TIMEOUT:
            print("Robot appears to be stuck! Activating recovery behavior.")
            # Return to rotation state for recovery
            state_ = 0
            stuck_time_ = 0
    else:
        stuck_time_ = 0  # Reset if moving
    
    # Update last position
    last_position_.x = position_.x
    last_position_.y = position_.y

def goal(desired_pose):
    """Navigate to goal point with obstacle avoidance"""
    global position_, yaw_, regions, state_, last_position_, stuck_time_
    
    # Initialize state for this goal
    state_ = 0  # Start with orientation alignment
    
    # Initialize stuck detection
    last_position_.x = position_.x
    last_position_.y = position_.y
    stuck_time_ = 0
    stuck_timer = rospy.Time.now()
    
    print("-------------------------")
    print("Goal point: ({:.2f}, {:.2f})".format(desired_pose.x, desired_pose.y))
    print("-------------------------")
    print("*** NAVIGATION STARTED ***")
    
    rate = rospy.Rate(10)  # 10Hz update rate
    while not rospy.is_shutdown():
        # Check for stuck condition (every second)
        if (rospy.Time.now() - stuck_timer).to_sec() > 1.0:
            check_if_stuck()
            stuck_timer = rospy.Time.now()
        
        # Check for obstacles - this has priority
        obstacle_detected = False
        if regions:
            front_danger = regions['front_L'] < OBSTACLE_DIST_THRESHOLD or regions['front_R'] < OBSTACLE_DIST_THRESHOLD
            side_danger = regions['fleft'] < OBSTACLE_DIST_THRESHOLD or regions['fright'] < OBSTACLE_DIST_THRESHOLD
            
            if front_danger or side_danger:
                obstacle_detected = True
                # Get obstacle avoidance command
                obstacle_cmd = obs.avoid_obstacle()
                
                if obstacle_cmd:
                    # Execute obstacle avoidance
                    cmd_pub.publish(obstacle_cmd)
                    print("Executing obstacle avoidance")
                    rate.sleep()
                    continue
        
        # Execute navigation behavior based on current state
        if state_ == 0:
            # Orientation correction state
            yaw_, position_, state_ = fix_yaw(desired_pose, yaw_, position_, state_)
        elif state_ == 1:
            # Move straight state
            yaw_, position_, state_ = go_straight_ahead(desired_pose, yaw_, position_, state_)
        elif state_ == 2:
            # Goal reached state
            stop()
            print("*** NAVIGATION COMPLETED ***")
            print("")
            break
        else:
            rospy.logerr('Unknown state: {}'.format(state_))
            stop()
            break
        
        # Calculate distance to goal (for display)
        distance_to_goal = math.sqrt(pow(desired_pose.y - position_.y, 2) + 
                                   pow(desired_pose.x - position_.x, 2))
                                   
        # Periodically display position info (every ~3 seconds)
        if (rospy.Time.now().to_sec() % 3) < 0.1:
            print("Distance to goal: {:.2f}m, State: {}".format(distance_to_goal, state_))
        
        rate.sleep()
    
    return True

def get_user_input():
    """Get goal coordinates from terminal input"""
    try:
        x = float(input("Enter X coordinate for goal: "))
        y = float(input("Enter Y coordinate for goal: "))
        return x, y
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        return get_user_input()

def navigation():
    """ Get goal input from terminal and trigger navigation """
    
    print("Single point navigation with terminal input")
    
    while not rospy.is_shutdown():
        # Get goal coordinates from user
        x, y = get_user_input()
        
        # Create desired position
        desired_position = Point()
        desired_position.x = x
        desired_position.y = y
        
        # Navigate to goal
        goal(desired_position)
        
        # Ask if user wants to continue
        try:
            continue_nav = input("Navigate to another point? (y/n): ")
            if continue_nav.lower() != 'y':
                break
        except:
            # If there's any error with input, default to exiting
            break
    
    print("Navigation terminated.")

def main():
    """ Initialize node and start navigation """
    global cmd_pub
    
    rospy.init_node('go_to_point_terminal')
    
    # Publishers
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Subscribers
    laser_sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)
    
    
    # Print information
    print("============= Go to Point (Terminal Input) ============")
    print("Publishers       : /cmd_vel")
    print("Subscribers      : /scan, /robot_pose_ekf/odom_combined")
    print("Linear_velocity  : "+ str(LINEAR_VELOCITY) +" m/s")
    print("Angular_velocity : "+ str(ANGULAR_VELOCITY) +" rad/s")
    print("=========================================================")
    print("")
    
    # Wait for sensors to connect
    sleep(1)
    
    return True

if __name__ == '__main__':
    try:
        if main():
            navigation()
    except rospy.ROSInterruptException:
        pass
