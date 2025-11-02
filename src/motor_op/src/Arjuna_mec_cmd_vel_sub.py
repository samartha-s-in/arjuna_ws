#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from STservo_sdk import *
from time import sleep

# Initial velocities for all four motors
FrontLeft_velocity = 0
FrontRight_velocity = 0
RearLeft_velocity = 0
RearRight_velocity = 0

# Motor controller parameters
BAUDRATE = 115200
DEVICENAME = '/dev/motor'
MOTOR_ACCL = 0

# Set up the port and packet handlers for the motor controller
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

# Open the port to communicate with the motor controller
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set the baudrate for communication
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def odom_callback(msg1):
    """
    Callback function that converts cmd_vel messages (linear and angular velocities) 
    into individual motor velocities for Mecanum wheels.
    """
    global FrontLeft_velocity, FrontRight_velocity, RearLeft_velocity, RearRight_velocity
    linear_vel = msg1.linear.x    # Forward/Backward velocity
    strafe_vel = msg1.linear.y    # Strafe (side to side) velocity
    angular_vel = msg1.angular.z # Rotation velocity around z-axis (turning)

    # Mecanum wheel kinematics equations
    FrontLeft_velocity = linear_vel + strafe_vel + angular_vel
    FrontRight_velocity = linear_vel - strafe_vel - angular_vel
    RearLeft_velocity = linear_vel - strafe_vel + angular_vel
    RearRight_velocity = linear_vel + strafe_vel - angular_vel

    # Scale velocities for motor controllers (you might need to adjust this scale factor)
    FrontLeft_velocity = int(FrontLeft_velocity * 10000)
    FrontRight_velocity = int(FrontRight_velocity * 10000)
    RearLeft_velocity = int(RearLeft_velocity * 10000)
    RearRight_velocity = int(RearRight_velocity * 10000)

    # Debugging print statements
    print(f"Received: linear_vel={linear_vel}, strafe_vel={strafe_vel}, angular_vel={angular_vel}")
    print(f"Calculated Motor Velocities: FL={FrontLeft_velocity}, FR={FrontRight_velocity}, RL={RearLeft_velocity}, RR={RearRight_velocity}")

def wheel_mode(Motor_ID):
    """Switch motor to wheel mode"""
    Result, Error = packetHandler.WheelMode(Motor_ID)

def run_motor(Motor_ID, Motor_Speed, Motor_Accel):
    """Set the motor speed and acceleration"""
    Result, Error = packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)

def enable_wheel_mode():
    """Enable wheel mode for all motors"""
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def run_robot():
    """
    Run the robot by setting the speed for all four Mecanum motors.
    The direction is based on the Mecanum kinematic equations.
    """
    global MOTOR_ACCL, FrontLeft_velocity, FrontRight_velocity, RearLeft_velocity, RearRight_velocity

    # Run motors based on calculated velocities
    run_motor(1, -FrontLeft_velocity, MOTOR_ACCL)  # Front Left motor
    run_motor(2, FrontRight_velocity, MOTOR_ACCL)  # Front Right motor
    run_motor(4, -RearLeft_velocity, MOTOR_ACCL)  # Rear Left motor
    run_motor(3, RearRight_velocity, MOTOR_ACCL)  # Rear Right motor

def main():
    """
    Main function to enable wheel mode and continuously update motor speeds
    based on cmd_vel subscriber messages.
    """
    enable_wheel_mode()  # Enable wheel mode for all motors

    rate = rospy.Rate(10)  # 10 Hz update rate
    while not rospy.is_shutdown():
        run_robot()  # Update robot's motor speeds
        rate.sleep()  # Sleep for the remainder of the cycle

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("Arjuna_cmd_vel_sub", anonymous=True)

    # Subscribe to the cmd_vel topic to receive Twist messages
    cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, odom_callback)

    print("\n")
    print("Publishers   : None")
    print("Subscribers  : cmd_vel")
    print("\n")

    # Sleep to ensure that ROS can set up everything
    sleep(2)

    # Run the robot control loop
    main()

    # Close the port when done
    portHandler.closePort()

