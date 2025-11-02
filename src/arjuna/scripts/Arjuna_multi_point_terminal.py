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
        if regions and not is_path_clear(regions):
            # Get obstacle avoidance command
            obstacle_cmd = avoid_obstacle(regions)
            
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

def get_waypoints():
    """Collect multiple waypoints from terminal input"""
    waypoints = []
    print("\n=== MULTI-POINT NAVIGATION WAYPOINT COLLECTION ===")
    print("Enter coordinates for each waypoint. When finished, enter 'done'.")
    
    waypoint_num = 1
    while True:
        print("\nWaypoint #{}: ".format(waypoint_num))
        try:
            user_input = raw_input("Enter X,Y coordinates (or 'done' to finish): ")
            
            if user_input.lower() == 'done':
                if len(waypoints) == 0:
                    print("No waypoints added. Please add at least one waypoint.")
                    continue
                else:
                    break
                    
            # Parse coordinates
            coords = user_input.split(',')
            if len(coords) != 2:
                print("Invalid format. Please use format: X,Y (e.g., 1.5,2.3)")
                continue
                
            x = float(coords[0].strip())
            y = float(coords[1].strip())
            
            # Create point and add to waypoints
            point = Point()
            point.x = x
            point.y = y
            waypoints.append(point)
            
            print("Waypoint #{} added: ({:.2f}, {:.2f})".format(waypoint_num, x, y))
            waypoint_num += 1
            
        except ValueError:
            print("Invalid input. Please enter numeric values or 'done'.")
            continue
    
    print("\n=== WAYPOINT COLLECTION COMPLETE ===")
    print("Total waypoints: {}".format(len(waypoints)))
    print("Waypoint coordinates:")
    for i, point in enumerate(waypoints):
        print("  #{}: ({:.2f}, {:.2f})".format(i+1, point.x, point.y))
    
    return waypoints

def multipoint_navigation():
    """Multi-point navigation main function"""
    # Collect waypoints from user
    waypoints = get_waypoints()
    
    if not waypoints:
        print("No waypoints to navigate. Exiting.")
        return
    
    print("\n=== STARTING MULTI-POINT NAVIGATION ===")
    print("Number of waypoints: {}".format(len(waypoints)))
    
    # Navigate through all waypoints
    for i, waypoint in enumerate(waypoints):
        print("\nNavigating to waypoint #{} of {}: ({:.2f}, {:.2f})".format(
            i+1, len(waypoints), waypoint.x, waypoint.y))
        
        # Navigate to this waypoint
        success = goal(waypoint)
        
        if not success:
            print("Navigation failed at waypoint #{}. Stopping.".format(i+1))
            return
        
        print("Waypoint #{} reached successfully.".format(i+1))
    
    print("\n=== MULTI-POINT NAVIGATION COMPLETED SUCCESSFULLY ===")
    print("All {} waypoints reached.".format(len(waypoints)))

def main():
    """Initialize node and start navigation"""
    global cmd_pub
    
    rospy.init_node('Arjuna_multipoint_terminal')
    
    # Publishers
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Subscribers
    laser_sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)
    
    # Print information
    print("\n===========================================")
    print("Arjuna Multi-Point Navigation (Terminal)")
    print("===========================================")
    print("Publishers       : /cmd_vel")
    print("Subscribers      : /scan, /robot_pose_ekf/odom_combined")
    print("Linear_velocity  : {:.2f} m/s".format(LINEAR_VELOCITY))
    print("Angular_velocity : {:.2f} rad/s".format(ANGULAR_VELOCITY))
    print("Obstacle threshold: {:.2f} m".format(OBSTACLE_DIST_THRESHOLD))
    print("")
    
    # Wait for sensors to connect
    sleep(1)
    
    return True

if __name__ == '__main__':
    try:
        if main():
            # Start multi-point navigation
            while not rospy.is_shutdown():
                multipoint_navigation()
                
                # Ask if user wants to plan another multi-point navigation
                try:
                    continue_nav = raw_input("\nPlan another multi-point navigation? (y/n): ")
                    if continue_nav.lower() != 'y':
                        print("Exiting multi-point navigation.")
                        break
                except:
                    # If there's any error with input, default to exiting
                    break
    except rospy.ROSInterruptException:
        pass
