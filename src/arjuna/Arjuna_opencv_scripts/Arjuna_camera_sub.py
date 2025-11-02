#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import time

# Custom CvBridge implementation to avoid cv_bridge compatibility issues
class CustomCvBridge:
    """
    Custom cv_bridge replacement for Python 3 compatibility
    """
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        """
        Convert an OpenCV image to a ROS image message
        """
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = 0
        
        if encoding == "bgr8":
            img_msg.step = cv_image.shape[1] * 3  # 3 bytes per pixel
            img_msg.data = cv_image.tobytes()
        elif encoding == "mono8":
            img_msg.step = cv_image.shape[1]
            img_msg.data = cv_image.tobytes()
        else:
            raise ValueError(f"Unsupported encoding: {encoding}")
        
        return img_msg
    
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

class OakDisplay:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('oak_display', anonymous=True)
        
        # Set up Custom CV bridge
        self.bridge = CustomCvBridge()
        
        # Variables
        self.rgb_frame = None
        self.depth_value = 0
        self.last_valid_depth = 0
        self.frame_count = 0
        self.start_time = rospy.get_time()
        self.fps = 0
        self.text_color = (0, 255, 0)  # Default text color (green)
        self.depth_text = "Depth: -- cm"
        
        # Subscribe to topics - simplified to just frames and depth
        rospy.Subscriber('/frames', Image, self.rgb_callback)
        rospy.Subscriber('/oak/depth_value', Float32, self.depth_callback)
        
        rospy.loginfo("Display node initialized")
    
    def rgb_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update FPS
            self.frame_count += 1
            current_time = rospy.get_time()
            elapsed = current_time - self.start_time
            
            if elapsed > 1.0:
                self.fps = self.frame_count / elapsed
                self.frame_count = 0
                self.start_time = current_time
            
            # Draw depth info on image
            self.draw_info()
            
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")
    
    def depth_callback(self, msg):
        # Store depth value
        self.depth_value = msg.data
        
        # Update text and color based on depth
        if self.depth_value < 0:
            self.depth_text = "Depth: Too close"
            self.text_color = (0, 0, 255)  # Red
        else:
            self.depth_text = f"Depth: {self.depth_value:.1f} cm"
            
            # Determine color based on distance
            if self.depth_value < 30:
                self.text_color = (0, 0, 255)  # Red for close objects
            elif self.depth_value < 100:
                self.text_color = (0, 255, 0)  # Green for medium distance
            else:
                self.text_color = (255, 255, 0)  # Yellow-blue for far objects
            
            # Update last valid depth if current is valid
            self.last_valid_depth = self.depth_value
    
    def draw_info(self):
        if self.rgb_frame is None:
            return
        
        # Create a copy of the frame
        display_frame = self.rgb_frame.copy()
        
        # Create a background rectangle for text
        cv2.rectangle(display_frame, (10, 10), (250, 45), (0, 0, 0), -1)
        
        # Display depth text
        cv2.putText(display_frame, self.depth_text, (15, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.text_color, 2)
        
        # Display FPS
        cv2.putText(display_frame, f"FPS: {self.fps:.1f}", (15, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Draw crosshair at center
        h, w = display_frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        cv2.line(display_frame, (center_x-10, center_y), (center_x+10, center_y), (0, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y-10), (center_x, center_y+10), (0, 255, 255), 1)
        
        # Show the frame
        cv2.imshow("OAK Camera Display", display_frame)
        cv2.waitKey(1)
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        display_node = OakDisplay()
        display_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
