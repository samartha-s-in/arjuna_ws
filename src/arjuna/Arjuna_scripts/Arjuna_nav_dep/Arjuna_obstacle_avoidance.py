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

# Constants
OBSTACLE_DIST_THRESHOLD = 0.5  # Distance threshold for obstacle detection (30cm)

# Global variables
regions = {}
pub = None

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

    take_action(regions)

def take_action(regions):
    """Decide on action based on laser scan regions"""
    global pub
    
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    # Check different obstacle configurations
    front_obstacle = regions['front_L'] < OBSTACLE_DIST_THRESHOLD or regions['front_R'] < OBSTACLE_DIST_THRESHOLD
    left_obstacle = regions['fleft'] < OBSTACLE_DIST_THRESHOLD or regions['left'] < OBSTACLE_DIST_THRESHOLD
    right_obstacle = regions['fright'] < OBSTACLE_DIST_THRESHOLD or regions['right'] < OBSTACLE_DIST_THRESHOLD
    
    state_description = ''
    
    # Decision tree for obstacle avoidance
    if front_obstacle and left_obstacle and right_obstacle:
        # Surrounded by obstacles - back up slightly then rotate
        state_description = 'case 1 - surrounded by obstacles'
        linear_x = -0.1
        angular_z = 0
    elif front_obstacle and left_obstacle:
        # Obstacles in front and left - turn right
        state_description = 'case 2 - obstacles front and left'
        linear_x = 0
        angular_z = -0.8  # Turn right
    elif front_obstacle and right_obstacle:
        # Obstacles in front and right - turn left
        state_description = 'case 3 - obstacles front and right'
        linear_x = 0
        angular_z = 0.8  # Turn left
    elif front_obstacle:
        # Only front obstacle - check which side has more space
        if regions['left'] > regions['right']:
            state_description = 'case 4 - obstacle front, turning left'
            linear_x = 0
            angular_z = 0.8  # Turn left (more space)
        else:
            state_description = 'case 5 - obstacle front, turning right'
            linear_x = 0
            angular_z = -0.8  # Turn right (more space)
    elif left_obstacle:
        # Left obstacle - slight right turn while moving
        state_description = 'case 6 - obstacle to the left'
        linear_x = 0.15
        angular_z = -0.4  # Slight right
    elif right_obstacle:
        # Right obstacle - slight left turn while moving
        state_description = 'case 7 - obstacle to the right'
        linear_x = 0.15
        angular_z = 0.4  # Slight left
    else:
        # No obstacles - clear path
        state_description = 'case 8 - no obstacles'
        linear_x = 0.2
        angular_z = 0
    
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def is_path_clear():
    """Check if path is clear of obstacles"""
    global regions
    
    if not regions:
        return True  # No LIDAR data yet, assume clear
    
    # Consider path clear if all regions are beyond threshold
    return (regions['front_L'] > OBSTACLE_DIST_THRESHOLD and 
            regions['front_R'] > OBSTACLE_DIST_THRESHOLD and
            regions['fleft'] > OBSTACLE_DIST_THRESHOLD and
            regions['fright'] > OBSTACLE_DIST_THRESHOLD)

def avoid_obstacle(angular_velocity):
    """Generate avoidance command based on obstacle configuration"""
    global regions
    
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
        twist_msg.angular.z = -angular_velocity
    elif front_obstacle and right_obstacle:
        # Obstacles in front and right - turn left
        print("Obstacles front and right - turning left")
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_velocity
    elif front_obstacle:
        # Only front obstacle - check which side has more space
        if regions['left'] > regions['right']:
            print("Front obstacle - turning left (more space)")
            twist_msg.linear.x = 0
            twist_msg.angular.z = angular_velocity
        else:
            print("Front obstacle - turning right (more space)")
            twist_msg.linear.x = 0
            twist_msg.angular.z = -angular_velocity
    elif left_obstacle:
        # Left obstacle - slight right turn while moving
        print("Left obstacle - adjusting right")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = -angular_velocity * 0.5
    elif right_obstacle:
        # Right obstacle - slight left turn while moving
        print("Right obstacle - adjusting left")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = angular_velocity * 0.5
    else:
        # No obstacles detected - clear path
        print("No obstacles detected - clear path")
        twist_msg = None
    
    return twist_msg

def main():
    global pub, regions
    
    rospy.init_node('obstacle_avoidance')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    print("Obstacle avoidance node initialized")
    print("Threshold distance: {} meters".format(OBSTACLE_DIST_THRESHOLD))
    
    rospy.spin()

if __name__ == '__main__':
    main()
