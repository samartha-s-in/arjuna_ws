#!/usr/bin/env python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
import numpy as np
from pyzbar.pyzbar import decode

# Custom CvBridge implementation for Python 3 compatibility
class CustomCvBridge:
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """
        Convert a ROS image message to an OpenCV image
        """
        if img_msg.encoding == "bgr8":
            cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, 3
            )
        elif img_msg.encoding == "mono8":
            cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width
            )
        else:
            raise ValueError(f"Unsupported encoding: {img_msg.encoding}")
        
        return cv_image

def callback(data):
    # Color detection boundaries in HSV
    lower = np.array([15, 150, 20])
    upper = np.array([35, 255, 255])

    br = CustomCvBridge()
    
    try:
        # Convert ROS Image to OpenCV image
        video = cv2.resize(br.imgmsg_to_cv2(data), (640, 420))

        # Detect QR codes first
        qr_codes = decode(video)
        for qr in qr_codes:
            qr_data = qr.data.decode("utf-8")
            if qr_data == "Arjuna":
                # Get QR code position
                x, y, w, h = qr.rect
                # Draw green rectangle for Arjuna QR code
                cv2.rectangle(video, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Color detection
        img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) # Converting BGR image to HSV format

        mask = cv2.inRange(img, lower, upper) # Masking the image to find our color

        mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Finding contours in mask image

        if len(mask_contours) != 0:
            
            for mask_contour in mask_contours:
                if cv2.contourArea(mask_contour) > 1000:

                    x, y, w, h = cv2.boundingRect(mask_contour)

                    rect = cv2.rectangle(video, (x, y), (x + w, y + h), (0, 0, 255), 3) #drawing rectangle

        cv2.imshow("original_video", video)

        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")
      
if __name__ == '__main__':
    rospy.init_node('video_sub_py', anonymous=True)
    rospy.Subscriber('/Arjuna_camera_frames', Image, callback)

    rospy.spin()
    cv2.destroyAllWindows()
