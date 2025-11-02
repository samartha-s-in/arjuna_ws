#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
import math
from time import sleep
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int8

#Docking orientation parameters 
BR_yaw_1    = 0
BR_yaw_2    = 0
quadrant_   = 0

#paras received from camera 
turn_right = 0 
turn_left  = 0
straight   = 0

Linear_velocity     = 0.15  #Linear velocity of the robot in m/s
Angular_velocity    = 0.4   #Angular velocity of the robot in rad/s

position_           = Point() #To store live odometry of the robot

goal_position_      = Point() #Stores X and Y coordinates of the goal point
goal_orientataion_  = Point() #Stores oriention(YAW) of the robot

yaw_precision_      = math.pi / 20 # +/- 2 degrees of error is allowed when robot is navigating in s straight line
dist_precision_     = 0.1       # Goal point radius in meters

yaw_                = 0  #Stores the live orientation of the robot 
state_              = 0  #Tells the state of the robot if state_ == 0: robot is rotating , if state_ == 1: Robot moving straight , if state == 2: Reached goal point

regions             = None #Lidar 360 degrees divided into different regions for obstacle avoidance
cmd_pub             = None #Publisher which publishes velocity for robot(arduino) through /cmd_vel topic 
mecanum_turn = 0

def clbk_laser(msg):
    global regions
    regions = {
        'front_L' : min(min(msg.ranges[0:130]), 10), 
        'fleft'   : min(min(msg.ranges[131:230]), 10),  
        'left'    : min(min(msg.ranges[231:280]), 10),
        'right'   : min(min(msg.ranges[571:620]), 10), 
        'fright'  : min(min(msg.ranges[621:720]), 10), 
        'front_R' : min(min(msg.ranges[721:850]), 10)
    }
  #  print(regions['point_right'])

def turn_right_callback(turn_right_msg):
    global turn_right
    turn_right = turn_right_msg.data
    #print("turn_right")

def turn_left_callback(turn_left_msg):
    global turn_left
    turn_left = turn_left_msg.data
    #print(turn_left)

def straight_callback(straight_msg):
    global straight
    straight = straight_msg.data
 ##   print(straight)


def fix_yaw(des_pos):
    global yaw_, cmd_pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = Angular_velocity if err_yaw > 0 else -Angular_velocity
    
    cmd_pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)
        
def go_straight_ahead(des_pos):
    global err_pos
    global yaw_, cmd_pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Linear_velocity
        #print(position_.x)
        cmd_pub.publish(twist_msg)
    else:
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def turn_left_odo():
    twist_msg = Twist()
    twist_msg.angular.z = Angular_velocity
    cmd_pub.publish(twist_msg)

def turn_right_odo():
    twist_msg = Twist()
    twist_msg.angular.z = -Angular_velocity
    cmd_pub.publish(twist_msg)

def move_straight():
    twist_msg = Twist()
    twist_msg.linear.x = Linear_velocity
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)
    #print("moveing straight ")

def turn_left_dock():
    mecanum_turn.publish(1)

def turn_right_dock():
    mecanum_turn.publish(-1)

def move_straight_dock():
    mecanum_turn.publish(2)

def stop_docking():
    mecanum_turn.publish(3)


def stop():
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    cmd_pub.publish(twist_msg)

def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,-------------------------

        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def goal(desired_pose): 
    global regions
    limiter = 0

    odom_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)

    print("-------------------------")
    print("Goal_point    : " + str(round(desired_pose.x , 1)) + "," + str(round(desired_pose.y , 1)))
    print("-------------------------")
    print("*** NAVIGATION STARTED ***")

    rate = rospy.Rate(10)
    while True:
        if state_ == 0:
            fix_yaw(desired_pose)
        elif state_ == 1:
            if regions['front_L'] > 0.2 and regions['front_L'] < 0.30 or regions['front_R'] > 0.20 and regions['front_R'] < 0.30 and regions['right'] > regions['left']:
                turn_right()
                print("turning right")
                limiter = 2
            elif regions['front_L'] > 0.2 and regions['front_L'] < 0.30 or regions['front_R'] > 0.20 and regions['front_R'] < 0.30 and regions['left'] > regions['right']:
                turn_left()
                print("turning left")
                limiter = 1
            elif regions['fright'] < 0.35 or regions['right'] < 0.40 and limiter == 1:
                move_straight()
                print("moveing straight ")
                limiter = 0
            elif regions['fleft'] < 0.35 or regions['left'] < 0.40 and limiter == 2:
                move_straight()
                print("moveing straight ")
                limiter = 0
            else:
                go_straight_ahead(desired_pose)
                
        elif state_ == 2:
            stop() 
            print("*** NAVIGATION COMPLETED ***")
            print("")
            break
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

def docking():
    global turn_right , turn_left , straight 
    global quadrant_ , yaw_ , cmd_pub
    
    x_goal = float(input("Enter x coordinate : "))
    y_goal = float(input("Enter Y coordinate : "))
    yaw_goal = float(input("Enter goal orientation : "))

    BR_yaw_1 = yaw_goal - 0.01
    BR_yaw_2 = yaw_goal + 0.01

    print(" x , Y and YAW of charging station are ")
    print(x_goal  ,  y_goal , yaw_goal)

    desired_position_1 = Point()
    desired_position_1.x = x_goal
    desired_position_1.y = y_goal
    goal(desired_position_1)
    print("Reached 1st goal")
    sleep(5)

    rate_orientation_1 = rospy.Rate(60)
    while True:
        if yaw_ > BR_yaw_2 :
            turn_right_odo()
        elif yaw_ < BR_yaw_1:
            turn_left_odo()
        elif yaw_ > BR_yaw_1 or yaw_ < BR_yaw_2:
            stop()
            break
        rate_orientation_1.sleep()
    print("Ready to dock")

    sleep(1)
    rate_open_cv_1 = rospy.Rate(10)
    while True:
        #print(turn_left , turn_right , straight)
        if straight == 0 and turn_left == 0 and turn_right == 0:
            stop_docking()
        elif turn_right == 1 and turn_left == 0 :
            turn_right_dock()
        elif turn_left == 1 and turn_right == 0:
            turn_left_dock()
        elif straight == 1 and turn_left == 0 and turn_right == 0 and regions['front_L'] > 0.30 or regions['front_R'] > 0.30:
            move_straight_dock()
            print("Moving straight")
        elif regions['front_L'] < 0.30 or regions['front_R'] < 0.30:
            stop_docking()
            print("Docking is successful")
            break
        rate_open_cv_1.sleep()


if __name__ == "__main__":

    rospy.init_node('go_to_point_code')
        
    cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    mecanum_turn   = rospy.Publisher('/mecanum_control', Int8, queue_size=1)
        
    sub_odom  = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, clbk_odom)
    
    laser_sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    sub_right = rospy.Subscriber('/turn_right' , Int8 , turn_right_callback)

    sub_left  = rospy.Subscriber('/turn_left' , Int8 , turn_left_callback)

    sub_straight = rospy.Subscriber('/straight' , Int8 , straight_callback)

    sleep(2)

    docking()
  
