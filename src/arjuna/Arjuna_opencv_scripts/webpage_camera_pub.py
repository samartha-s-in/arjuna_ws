#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import depthai as dai
import numpy as np
from pyzbar.pyzbar import decode

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
    
    # Set preview size to 420x380 to match the original code
    camRgb.setPreviewSize(420, 380)
    
    # Linking
    camRgb.preview.link(xoutRgb.input)
    
    return pipeline

def image_publisher():
    # Set up OAK-D camera
    pipeline = setup_oak_pipeline()
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        rospy.loginfo("Connected to OAK-D Lite camera")
        
        # Output queue
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # Get frame from OAK-D camera
            in_rgb = q_rgb.get()
            frame = in_rgb.getCvFrame()
            
            if frame is None:
                rospy.logerr("Failed to get frame")
                continue
            
            # Check for QR codes to add green box for "Arjuna"
            qr_codes = decode(frame)
            for qr in qr_codes:
                qr_data = qr.data.decode("utf-8")
                if qr_data == "Arjuna":
                    # Get QR code position
                    x, y, w, h = qr.rect
                    # Draw green rectangle for Arjuna QR code
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            # Create compressed image message
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = cv2.imencode('.jpg', frame)[1].tobytes()

            # Publish
            Frames_pub.publish(msg)
            print("Publishing")

            # Display the frame
            cv2.imshow("Web Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('image_publisher', anonymous=True)
    Frames_pub = rospy.Publisher('/frames', CompressedImage, queue_size=10)
    
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")
    finally:
        cv2.destroyAllWindows()
