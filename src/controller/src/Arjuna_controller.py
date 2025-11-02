#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions for the user, updated for the new stateful controls
msg = """
Control Your Robot! (Stateful Mode)
---------------------------
The robot will continue moving with the last command until you issue a new one.
Press SPACE to stop the robot.

Current Mode: Differential

--- Differential Controls ---
   ↑    ^    →   
      <   >
   ↓    v    ←

   ^ / v: Forward / Backward
   ← / →: Turn Left / Turn Right
   ↑ / →: Forward-Turn Left / Right
   ↓ / ←: Backward-Turn Left / Right
   
--- Mecanum Controls ---
   w    s    a
   ↑    ↓    d
   q    z    e

   w/s: Forward / Backward
   a/d: Strafe Left / Right
   q/e: Diagonal Forward Left / Right
   z/c: Diagonal Backward Left / Right
   x:   Turn in place (use j/l to set direction)

--- Global Controls ---
- SPACE: Emergency Stop
- t: Toggle between Differential and Mecanum modes.
- i/k: Increase/Decrease linear speed by 10%
- j/l: Increase/Decrease angular speed by 10%
- CTRL-C to quit
"""

# Key bindings for speed adjustments
speedBindings = {
    'i': (1.1, 1),
    'k': (0.9, 1),
    'j': (1, 1.1),
    'l': (1, 0.9),
}

# Key bindings for differential drive mode (4 values per key now)
differentialBindings = {
    'up':    (1, 0, 0, 0),  # Forward (Move along x-axis)
    'down':  (-1, 0, 0, 0), # Backward (Move along -x-axis)
    'left':  (0, 0, 0, 1),  # Turn Left (Rotate around z-axis)
    'right': (0, 0, 0, -1), # Turn Right (Rotate around -z-axis)
    'u':     (1, 0, 0, 1),  # Forward-Left (Diagonal with turn)
    'o':     (1, 0, 0, -1), # Forward-Right (Diagonal with turn)
    'm':     (-1, 0, 0, 1), # Backward-Left (Diagonal with turn)
    '.':     (-1, 0, 0, -1), # Backward-Right (Diagonal with turn)
}

# Key bindings for mecanum drive mode (4 values per key now)
mecanumBindings = {
    'w': (1, 0, 0, 0),   # Forward
    's': (-1, 0, 0, 0),  # Backward
    'a': (0, -1, 0, 0),   # Strafe Left
    'd': (0, 1, 0, 0),  # Strafe Right
    'q': (1, -1, 0, 0),   # Forward-Left (Diagonal)
    'e': (1, 1, 0, 0),  # Forward-Right (Diagonal)
    'z': (-1, -1, 0, 0),  # Backward-Left (Diagonal)
    'c': (-1, 1, 0, 0), # Backward-Right (Diagonal)
    'x': (0, 0, 0, 1),   # Turn Left (in place)
}

def getKey():
    """
    Gets a single key press from the user.
    Handles multi-byte sequences for arrow keys.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        # Check for escape sequence (arrow keys)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':
                key = 'up'
            elif extra == '[B':
                key = 'down'
            elif extra == '[C':
                key = 'right'
            elif extra == '[D':
                key = 'left'
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """Returns a string displaying the current speed and turn rate."""
    return "currently:\tspeed {}\tturn {}".format(round(speed, 2), round(turn, 2))

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('arjuna_c')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Default parameters
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    
    # Robot state variables (x, y, th)
    # These will persist until a new command is given
    x, y, z, th = 0, 0, 0, 0
    mode = 'differential'  # Start in differential mode

    try:
        print(msg)
        print(vels(speed, turn))

        while not rospy.is_shutdown():
            key = getKey()

            # Handle global keys first
            if key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))
            elif key == 't':
                # Stop the robot when changing modes
                x, y, z, th = 0, 0, 0, 0
                if mode == 'differential':
                    mode = 'mecanum'
                    print("Mode switched to: Mecanum")
                else:
                    mode = 'differential'
                    print("Mode switched to: Differential")
            elif key == ' ':
                # Universal stop key, resets the state
                x, y, z, th = 0, 0, 0, 0
                print("EMERGENCY STOP")
            elif key == '\x03':  # CTRL-C
                break
            else:
                # Handle mode-specific keys
                if mode == 'differential':
                    if key in differentialBindings:
                        x = differentialBindings[key][0]
                        y = differentialBindings[key][1]
                        z = differentialBindings[key][2]  # z-axis (vertical) linear speed
                        th = differentialBindings[key][3]  # angular z (rotation)
                
                elif mode == 'mecanum':
                    if key in mecanumBindings:
                        x = mecanumBindings[key][0]
                        y = mecanumBindings[key][1]
                        z = mecanumBindings[key][2]  # z-axis (vertical) linear speed
                        th = mecanumBindings[key][3]  # angular z (rotation)

            # Create and publish the Twist message from the current state
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed  # For vertical movement (e.g., lift, etc.)
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Ensure the robot is stopped when the script is terminated
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

