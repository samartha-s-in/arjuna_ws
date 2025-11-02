#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from std_msgs.msg import Int8
from time import sleep
import depthai as dai

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

def process():
    # Set up OAK-D camera
    pipeline = setup_oak_pipeline()
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        print("Connected to OAK-D Lite camera")
        
        # Output queue
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        while not rospy.is_shutdown():
            # Get frame from OAK-D camera
            in_rgb = q_rgb.get()
            Frame = in_rgb.getCvFrame()
            
            if Frame is None:
                print("Failed to capture frame")
                continue

            # Try direct decoding first (faster)
            decoded_image = decode(Frame)
            
            # If no QR codes found, try with preprocessing
            if not decoded_image:
                # Convert to grayscale
                gray = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
                # Apply Gaussian blur
                blurred = cv2.GaussianBlur(gray, (7, 7), 0)
                # Apply adaptive threshold
                adaptive_thresh = cv2.adaptiveThreshold(
                    blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                # Try decoding again
                decoded_image = decode(adaptive_thresh)

            qr_detected = False
            rows, columns, _ = Frame.shape
            y_tolerance_1 = int(columns / 2.4) 
            y_tolerance_2 = int(columns / 1.61) 
            y_center = int(columns / 2.33)

            for barcode in decoded_image:
                qr_detected = True
                string_data = barcode.data.decode("utf-8")
                print(f"Detected QR code: {string_data}")
                
                if string_data == "Arjuna":
                    x, y, w, h = barcode.rect
                    y_medium = int(x+w/2)
                    
                    # Drawing GREEN rectangle for Arjuna QR code
                    rect = cv2.rectangle(Frame, (x, y), (x + w, y + h), (0, 255, 0), 3) #drawing green rectangle
                    cv2.line(Frame, (y_medium,0), (y_medium,480), (0,0,255), 1)

                    if y_medium > y_tolerance_2:
                        print("turning right")
                        right = 1
                        left = 0
                        straight = 0
                        right_pub.publish(right)
                        left_pub.publish(left)
                        straight_pub.publish(straight)
                    elif y_medium < y_tolerance_1:
                        print("turning left")
                        left = 1
                        right = 0
                        straight = 0
                        left_pub.publish(left)
                        right_pub.publish(right)
                        straight_pub.publish(straight)
                    elif y_medium > y_tolerance_1 and y_medium < y_tolerance_2:
                        print("moving straight")
                        straight = 1
                        left = 0
                        right = 0
                        right_pub.publish(right)
                        left_pub.publish(left)
                        straight_pub.publish(straight)
                else:
                    # For non-Arjuna QR codes, don't draw any box
                    pass

            if not qr_detected:
                print("NO QR RECOGNISED")
                right = 0
                left = 0
                straight = 0
                right_pub.publish(right)
                left_pub.publish(left)
                straight_pub.publish(straight)

            y_tol_line_1 = cv2.line(Frame, (y_tolerance_2,0), (y_tolerance_2,480), (255,0,0), 1)
            y_tol_line_2 = cv2.line(Frame, (y_tolerance_1,0), (y_tolerance_1,480), (255,0,0), 1)
            cv2.imshow("image", Frame)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break


if __name__ == '__main__':
    rospy.init_node('Arjuna_camera_subscriber', anonymous=True)
    right_pub = rospy.Publisher('turn_right', Int8, queue_size = 10)
    left_pub = rospy.Publisher('turn_left', Int8, queue_size = 10)
    straight_pub = rospy.Publisher('straight', Int8, queue_size = 10)

    sleep(1)
    
    try:
        process()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
