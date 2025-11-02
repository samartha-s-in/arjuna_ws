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
import math
import re
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from std_msgs.msg import String
from time import sleep

# Import functions from Newrro_Navigation
from Newrro_Navigation import normalize_angle, stop

# Navigation parameters
LINEAR_VELOCITY = 0.2      # Linear velocity of the robot in m/s
ANGULAR_VELOCITY = 0.5     # Angular velocity of the robot in rad/s
YAW_PRECISION = math.pi / 90  # +/- 2 degrees precision for orientation
DIST_PRECISION = 0.02       # Distance precision (2cm)

# Global variables
position_ = Point()          # Current robot position
yaw_ = 0                     # Current robot orientation
initial_position_ = Point()  # Starting position for distance measurement
initial_yaw_ = 0             # Starting orientation for turn measurement
cmd_pub = None               # Publisher for velocity commands
feedback_pub = None          # Publisher for feedback

# Command processing flags
executing_command = False    # Flag to indicate a command is being executed
current_command = None       # Currently executing command

# Conversion factors
METERS_TO_CM = 100.0
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

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

def send_feedback(text):
    """Send feedback (could be spoken or displayed)"""
    global feedback_pub
    
    # Publish feedback if available
    if feedback_pub:
        msg = String()
        msg.data = text
        feedback_pub.publish(msg)
    
    # Always print to console
    print(text)

def move_straight(distance):
    """Move forward or backward by a specific distance in meters"""
    global position_, initial_position_, executing_command
    
    # Store initial position
    initial_position_.x = position_.x
    initial_position_.y = position_.y
    
    # Set direction (positive = forward, negative = backward)
    direction = 1 if distance > 0 else -1
    target_distance = abs(distance)
    
    # Notify start of movement
    direction_str = "forward" if direction > 0 else "backward"
    send_feedback("Moving {} {} meters".format(direction_str, target_distance))
    
    rate = rospy.Rate(10)  # 10Hz update rate
    
    while not rospy.is_shutdown() and executing_command:
        # Calculate distance moved so far
        dist_moved = math.sqrt(pow(position_.x - initial_position_.x, 2) + 
                              pow(position_.y - initial_position_.y, 2))
        
        # Check if we've reached the target distance
        if dist_moved >= target_distance - DIST_PRECISION:
            stop()
            send_feedback("Moved {} {} meters".format(direction_str, target_distance))
            executing_command = False
            return True
        
        # Create movement command
        twist_msg = Twist()
        twist_msg.linear.x = direction * LINEAR_VELOCITY
        
        # Publish command
        cmd_pub.publish(twist_msg)
        
        # Sleep to maintain rate
        rate.sleep()
    
    # If we get here, command was interrupted
    stop()
    return False

def turn(degrees):
    """Turn by a specific angle in degrees (positive = left/CCW, negative = right/CW)"""
    global yaw_, initial_yaw_, executing_command
    
    # Convert degrees to radians
    angle_rad = degrees * DEG_TO_RAD
    
    # Store initial orientation
    initial_yaw_ = yaw_
    
    # Calculate target yaw
    target_yaw = normalize_angle(initial_yaw_ + angle_rad)
    
    # Determine direction name for feedback
    direction_str = "left" if degrees > 0 else "right"
    abs_degrees = abs(degrees)
    
    # Notify start of turn
    send_feedback("Turning {} {} degrees".format(direction_str, abs_degrees))
    
    rate = rospy.Rate(10)  # 10Hz update rate
    
    while not rospy.is_shutdown() and executing_command:
        # Calculate the error (shortest path to target)
        error = normalize_angle(target_yaw - yaw_)
        
        # Check if we've reached the target orientation
        if abs(error) <= YAW_PRECISION:
            stop()
            send_feedback("Turned {} {} degrees".format(direction_str, abs_degrees))
            executing_command = False
            return True
        
        # Create movement command
        twist_msg = Twist()
        
        # Proportional control - slower when closer to target
        k_p = 1.0  # Proportional gain
        twist_msg.angular.z = k_p * error
        
        # Clamp to max velocity
        if twist_msg.angular.z > ANGULAR_VELOCITY:
            twist_msg.angular.z = ANGULAR_VELOCITY
        elif twist_msg.angular.z < -ANGULAR_VELOCITY:
            twist_msg.angular.z = -ANGULAR_VELOCITY
        
        # Publish command
        cmd_pub.publish(twist_msg)
        
        # Sleep to maintain rate
        rate.sleep()
    
    # If we get here, command was interrupted
    stop()
    return False

def extract_number(text, default=1.0):
    """Extract a number from text, return default if not found"""
    # Look for numbers like "1", "1.5", "0.5", ".5"
    matches = re.findall(r'(\d+\.\d+|\.\d+|\d+)', text)
    if matches:
        return float(matches[0])
    return default

def process_move_command(command):
    """Process a movement command"""
    global executing_command
    
    if not executing_command:
        executing_command = True
        
        # Default distance
        distance = 1.0  # meter
        
        # Extract distance if specified
        distance_value = extract_number(command)
        
        # Check direction
        if "back" in command or "backward" in command:
            distance = -distance_value
        else:
            distance = distance_value
        
        # Execute movement
        move_straight(distance)
    else:
        send_feedback("Already executing a command. Please wait.")

def process_turn_command(command):
    """Process a turn command"""
    global executing_command
    
    if not executing_command:
        executing_command = True
        
        # Default angle (90 degrees)
        angle = 90.0
        
        # Extract angle if specified
        angle_value = extract_number(command)
        if "deg" in command or "degree" in command:
            # If degrees are explicitly mentioned, use the extracted value
            angle = angle_value
        
        # Check direction
        if "right" in command:
            angle = -angle
        # else: left turn (positive angle)
        
        # Execute turn
        turn(angle)
    else:
        send_feedback("Already executing a command. Please wait.")

def process_combined_command(command):
    """Process a combined movement and turn command"""
    global executing_command
    
    if not executing_command:
        # Parse turn part
        turn_match = re.search(r'turn (left|right)( \d+)?( degrees?)?', command)
        if turn_match:
            turn_dir = turn_match.group(1)
            turn_angle = 90.0  # Default
            
            # Extract angle if specified
            if turn_match.group(2):
                turn_angle = float(turn_match.group(2).strip())
            
            if turn_dir == "right":
                turn_angle = -turn_angle
        else:
            return False  # No turn part found
        
        # Parse move part
        move_match = re.search(r'(move|go) (forward|backward)( \d+(\.\d+)?)?( meters?)?', command)
        if move_match:
            move_dir = move_match.group(2)
            move_dist = 1.0  # Default
            
            # Extract distance if specified
            if move_match.group(3):
                move_dist = float(move_match.group(3).strip())
            
            if move_dir == "backward":
                move_dist = -move_dist
        else:
            return False  # No move part found
        
        # Execute command sequence
        executing_command = True
        
        # First turn
        turn_success = turn(turn_angle)
        
        # If turn successful, then move
        if turn_success:
            # Small pause between commands
            rospy.sleep(0.5)
            move_straight(move_dist)
        
        return True
    else:
        send_feedback("Already executing a command. Please wait.")
        return True

def process_command(command_text):
    """Process a voice command"""
    global executing_command, current_command
    
    # Convert to lowercase and trim whitespace
    command = command_text.lower().strip()
    
    # Stop command has highest priority
    if "stop" in command or "halt" in command or "cancel" in command:
        stop()
        executing_command = False
        send_feedback("Stopped")
        return
    
    # Skip processing if already executing a command
    if executing_command:
        send_feedback("Already executing a command. Please wait or say 'stop' to cancel.")
        return
    
    # Store current command
    current_command = command
    print("Processing command: " + command)
    
    # Try to match combined command first
    if "and" in command:
        parts = command.split("and")
        if len(parts) >= 2:
            # Check if first part is turn and second is move
            if ("turn" in parts[0] and ("move" in parts[1] or "go" in parts[1])):
                if process_combined_command(command):
                    return
            
            # Check if first part is move and second is turn
            if (("move" in parts[0] or "go" in parts[0]) and "turn" in parts[1]):
                # Swap the order - always turn first then move
                reversed_command = parts[1] + " and " + parts[0]
                if process_combined_command(reversed_command):
                    return
    
    # Try to match turn command
    if "turn" in command or "rotate" in command:
        process_turn_command(command)
        return
    
    # Try to match move command
    if "move" in command or "go" in command or "forward" in command or "backward" in command:
        process_move_command(command)
        return
    
    # No match found
    send_feedback("I didn't understand that command. Try saying: 'move forward 1 meter', 'turn right 90 degrees', or 'stop'.")

def voice_command_callback(msg):
    """Callback for voice commands from ROS topic"""
    process_command(msg.data)

def main():
    """Initialize node and start voice movement controller"""
    global cmd_pub, feedback_pub
    
    rospy.init_node('Arjuna_voice_movement')
    
    # Publishers
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    feedback_pub = rospy.Publisher('/voice_feedback', String, queue_size=1)
    
    # Subscribers
    odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)
    voice_sub = rospy.Subscriber('/voice_command', String, voice_command_callback)
    
    # Print information
    print("\n===========================================")
    print("Arjuna Voice Movement Controller")
    print("===========================================")
    print("Publishers       : /cmd_vel, /voice_feedback")
    print("Subscribers      : /robot_pose_ekf/odom_combined, /voice_command")
    print("")
    print("Listening for commands:")
    print("  - 'move forward 1 meter'")
    print("  - 'move backward 0.5 meters'")
    print("  - 'turn left 90 degrees'")
    print("  - 'turn right 45 degrees'")
    print("  - 'turn left and move forward 1 meter'")
    print("  - 'stop'")
    print("===========================================")
    print("")
    
    # Wait for sensors to connect
    sleep(1)
    
    send_feedback("Voice movement controller activated and ready for commands")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
