#!/usr/bin/env python3

import rospy
import depthai as dai
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from pyzbar.pyzbar import decode
from time import sleep

# Motion control parameters
Linear_velocity = 0.15    # Linear velocity of the robot in m/s
Angular_velocity = 0.7    # Angular velocity of the robot in rad/s
regions = None            # LiDAR regions

# QR code tracking variables
turn_right = 0
turn_left = 0
straight = 0

def setup_oak_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # RGB Camera
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    
    # RGB camera properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(30)
    
    # Set preview size to 640x480 to match the original code
    camRgb.setPreviewSize(640, 480)
    
    # Linking
    camRgb.preview.link(xoutRgb.input)
    
    return pipeline

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

def turn_left_velo():
    global Angular_velocity, cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = Angular_velocity
    cmd_pub.publish(twist_msg)

def turn_right_velo():
    global Angular_velocity, cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = -Angular_velocity
    cmd_pub.publish(twist_msg)

def move_straight_velo():
    global Linear_velocity, cmd_pub
    twist_msg = Twist()
    twist_msg.linear.x = Linear_velocity
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)

def stop():
    global cmd_pub
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_pub.publish(twist_msg)

def preprocess_image(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise and improve edge detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use adaptive thresholding to emphasize QR code patterns
    adaptive_thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    
    return adaptive_thresh

def process():
    global turn_right, turn_left, straight, regions
    
    # Set up OAK-D camera
    pipeline = setup_oak_pipeline()
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        print("Connected to OAK-D Lite camera")
        
        # Output queue
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        rate = rospy.Rate(10)  # 10 Hz processing rate
        
        # Wait for initial LiDAR data
        while regions is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for LiDAR data...")
            rate.sleep()
            
        rospy.loginfo("LiDAR data received, starting QR tracking")
        
        while not rospy.is_shutdown():
            # Get frame from OAK-D camera
            in_rgb = q_rgb.get()
            frame = in_rgb.getCvFrame()
            
            if frame is None:
                rospy.logwarn("Failed to capture frame")
                continue
                
            # Process the frame for QR code detection
            rows, columns, _ = frame.shape
            y_tolerance_1 = int(columns / 2.3)
            y_tolerance_2 = int(columns / 1.71)
            
            # Draw tolerance lines
            cv2.line(frame, (y_tolerance_2, 0), (y_tolerance_2, 480), (255, 0, 0), 1)
            cv2.line(frame, (y_tolerance_1, 0), (y_tolerance_1, 480), (255, 0, 0), 1)
            
            # Try direct decoding first (faster)
            decoded_image = decode(frame)
            
            # If no QR codes found, try with preprocessing
            if not decoded_image:
                # Convert to grayscale and apply preprocessing
                processed_frame = preprocess_image(frame)
                # Try decoding again
                decoded_image = decode(processed_frame)
            
            qr_detected = False
            
            # Reset control flags for new frame processing
            turn_right = 0
            turn_left = 0
            straight = 0
            
            # Process detected QR codes
            for barcode in decoded_image:
                qr_detected = True
                string_data = barcode.data.decode("utf-8")
                
                if string_data == "Arjuna":
                    x, y, w, h = barcode.rect
                    y_medium = int(x + w / 2)
                    
                    # Draw green rectangle for Arjuna QR code
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.line(frame, (y_medium, 0), (y_medium, 480), (0, 0, 255), 1)
                    
                    # Determine movement based on QR code position
                    if y_medium > y_tolerance_2:
                        turn_right = 1
                        left_pub.publish(0)
                        right_pub.publish(1)
                        straight_pub.publish(0)
                        cv2.putText(frame, "Turning Right", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    elif y_medium < y_tolerance_1:
                        turn_left = 1
                        left_pub.publish(1)
                        right_pub.publish(0)
                        straight_pub.publish(0)
                        cv2.putText(frame, "Turning Left", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        straight = 1
                        straight_pub.publish(1)
                        left_pub.publish(0)
                        right_pub.publish(0)
                        cv2.putText(frame, "Moving Straight", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    # For non-Arjuna QR codes, don't draw any box
                    pass
            
            if not qr_detected:
                cv2.putText(frame, "NO QR RECOGNISED", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                right_pub.publish(0)
                left_pub.publish(0)
                straight_pub.publish(0)
            
            # Execute movement based on LiDAR and QR detection
            if regions['front_L'] > 0.30 and regions['front_R'] > 0.30:
                if turn_right == 1 and turn_left == 0:
                    turn_right_velo()
                    cv2.putText(frame, "Movement: Right", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                elif turn_left == 1 and turn_right == 0:
                    turn_left_velo()
                    cv2.putText(frame, "Movement: Left", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                elif straight == 1 and turn_left == 0 and turn_right == 0:
                    move_straight_velo()
                    cv2.putText(frame, "Movement: Straight", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                else:
                    stop()
                    cv2.putText(frame, "Movement: Stop", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                stop()
                cv2.putText(frame, "OBSTACLE DETECTED", (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "Movement: Stop", (10, 90), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display LiDAR readings
            cv2.putText(frame, f"Front L: {regions['front_L']:.2f}m", (columns-200, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Front R: {regions['front_R']:.2f}m", (columns-200, 60), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show the frame
            cv2.imshow("OAK-D QR Tracking", frame)
            cv2.waitKey(1)
            
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('OAK_QR_Tracking')
    
    # Publishers for robot control
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    right_pub = rospy.Publisher('turn_right', Int8, queue_size=10)
    left_pub = rospy.Publisher('turn_left', Int8, queue_size=10)
    straight_pub = rospy.Publisher('straight', Int8, queue_size=10)
    
    # Subscribe to LiDAR data
    laser_sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    # Wait for ROS to initialize and LiDAR to connect
    sleep(2)
    
    try:
        process()
    except Exception as e:
        rospy.logerr(f"Error in QR tracking: {e}")
    finally:
        # Make sure the robot stops if the node shuts down
        stop()
        cv2.destroyAllWindows()
