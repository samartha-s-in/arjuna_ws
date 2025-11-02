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

Linear_velocity     = 0.1   #Linear velocity of the robot in m/s
Angular_velocity    = 0.8    #Angular velocity of the robot in rad/s

position_           = Point() #To store live odometry of the robot

yaw_precision_      = math.pi / 20 # +/- 2 degrees of error is allowed when robot is navigating in s straight line
dist_precision_     = 0.05         # Goal point radius in meters

regions             = None #Lidar 360 degrees divided into different regions for obstacle avoidance
cmd_pub             = None #Publisher which publishes velocity for robot(arduino) through /cmd_vel topic 

yaw_                = 0  #Stores the live orientation of the robot 
state_              = 0  #Tells the state of the robot if state_ == 0: robot is rotating , if state_ == 1: Robot moving straight , if state == 2: Reached goal point

x1_goal              = 0  #Stores X coordinate of the goal point
y1_goal              = 0  #Stores Y coordinate of the goal point

x2_goal              = 0  #Stores X coordinate of the goal point
y2_goal              = 0  #Stores Y coordinate of the goal point

x3_goal              = 0  #Stores X coordinate of the goal point
y3_goal              = 0  #Stores Y coordinate of the goal point


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


def clbk_odom(msg):
    global position_
    global yaw_

    position_ = msg.pose.pose.position  # position

    quaternion = (                      # yaw
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def move_straight():
    global Linear_velocity

    twist_msg = Twist()
    twist_msg.linear.x = Linear_velocity
    cmd_pub.publish(twist_msg)

def turn_left():
    global Angular_velocity

    twist_msg = Twist()
    twist_msg.angular.z = Angular_velocity
    cmd_pub.publish(twist_msg)

def turn_right():
    global Angular_velocity

    twist_msg = Twist()
    twist_msg.angular.z = -Angular_velocity
    cmd_pub.publish(twist_msg)

def stop():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def fix_yaw(des_pos):
    global yaw_, cmd_pub, yaw_precision_, state_
    global Angular_velocity

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
    global Linear_velocity

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Linear_velocity
        cmd_pub.publish(twist_msg)
    else:
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

def go_straight_ahead(des_pos):
    global err_pos
    global yaw_, cmd_pub, yaw_precision_, state_
    global Linear_velocity

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Linear_velocity
        cmd_pub.publish(twist_msg)
    else:
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

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
            change_state(0)
            print("*** NAVIGATION COMPLETED ***")
            print("")
            break
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()


def navigation():
    global x1_goal , y1_goal , x2_goal , y2_goal , x3_goal , y3_goal
    x1_goal = float(input("Enter 1st goal x coordinate : "))
    y1_goal = float(input("Enter 1st goal Y coordinate : "))
    print("")

    x2_goal = float(input("Enter 2nd goal x coordinate : "))
    y2_goal = float(input("Enter 2nd goal Y coordinate : "))
    print("")

    x3_goal = float(input("Enter 3rd goal x coordinate : "))
    y3_goal = float(input("Enter 3rd goal Y coordinate : "))
    print("")

    ########## GOAL 1 ############
    desired_position_1= Point()
    desired_position_1.x = x1_goal
    desired_position_1.y = y1_goal

    goal(desired_position_1)
    sleep(3)

    ############# GOAL 2 ##################
    desired_position_2= Point()
    desired_position_2.x = x2_goal
    desired_position_2.y = y2_goal

    goal(desired_position_2)
    sleep(3)

    ############# GOAL 3 ##################
    desired_position_3= Point()
    desired_position_3.x = x3_goal
    desired_position_3.y = y3_goal

    goal(desired_position_3)

def main():
    global Linear_velocity , Angular_velocity , cmd_pub

    rospy.init_node('NR_B1')
    
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    laser_sub = rospy.Subscriber('/scan', LaserScan, clbk_laser) 

    print("Publishers       : /cmd_pub")
    print("Subscribers      : /scan , /robot_pose_ekf/odom_combined")
    print("Linear_velocity  : " + str(Linear_velocity) + " m/s")
    print("Angular_velocity : " + str(Angular_velocity) + " rad/s") 
    print("")
    sleep(1)

if __name__ == '__main__':
    main()
    navigation()

