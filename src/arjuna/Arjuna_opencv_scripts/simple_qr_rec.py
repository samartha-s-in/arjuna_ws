#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from std_msgs.msg import Int8, String
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
    
    # Set preview size to 640x480
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
            frame = in_rgb.getCvFrame()
            
            if frame is None:
                print("Failed to capture frame")
                continue
                
            # Try direct decoding first (faster)
            qr_codes = decode(frame)
            
            # If no QR codes found, try with preprocessing
            if not qr_codes:
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Apply Gaussian blur to reduce noise
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # Apply adaptive thresholding to get better QR code contrast
                thresh = cv2.adaptiveThreshold(
                    blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                
                # Try decoding again with preprocessed image
                qr_codes = decode(thresh)
            
            # Process detected QR codes
            qr_detected = False
            
            for qr in qr_codes:
                qr_detected = True
                
                # Get QR code data
                qr_data = qr.data.decode('utf-8')
                qr_type = qr.type
                
                # Get QR code position
                x, y, w, h = qr.rect
                
                # Draw a green box only if the QR code data is "Arjuna"
                if qr_data == "Arjuna":
                    # Draw green rectangle around QR code
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Draw QR code data on frame
                    cv2.putText(frame, f"Data: {qr_data}", (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Calculate center point
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Draw center point
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Publish QR code data
                qr_data_pub.publish(qr_data)
                
                print(f"Detected QR Code: {qr_data}, Type: {qr_type}")
            
            if not qr_detected:
                # Publish empty string if no QR code detected
                qr_data_pub.publish("")
            
            # Display the frame with QR code information
            cv2.imshow("QR Code Detection", frame)
            
            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == '__main__':
    rospy.init_node('oak_qr_detector', anonymous=True)
    
    # Publisher for QR code data
    qr_data_pub = rospy.Publisher('qr_data', String, queue_size=1)
    
    try:
        process()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        cv2.destroyAllWindows()
