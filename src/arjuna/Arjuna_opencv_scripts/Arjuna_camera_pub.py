#!/usr/bin/env python3

import rospy
import depthai as dai
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

def main():
    # Initialize ROS node
    rospy.init_node('oak_publisher', anonymous=True)
    
    # Set up Custom CV bridge
    bridge = CustomCvBridge()
    
    # Create publishers - simplified to just frames and depth
    frames_pub = rospy.Publisher('/frames', Image, queue_size=2)
    depth_pub = rospy.Publisher('/oak/depth_value', Float32, queue_size=2)
    
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
    camRgb.setVideoSize(640, 480)
    camRgb.setFps(30)
    
    # Linking
    camRgb.video.link(xoutRgb.input)
    
    # Stereo camera setup
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutDepth = pipeline.create(dai.node.XLinkOut)
    
    xoutDepth.setStreamName("depth")
    
    # Properties for mono cameras
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setFps(30)
    
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setFps(30)
    
    # StereoDepth configuration
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)  # Enable for closer range (<30cm)
    
    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(xoutDepth.input)
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        rospy.loginfo("Connected to OAK-D Lite camera")
        
        # Output queues
        q_rgb = device.getOutputQueue(name="rgb", maxSize=2, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=2, blocking=False)
        
        # Set minimum reliable depth (in mm)
        min_depth = 100  # 10cm
        
        # Variables to store last valid depth
        last_valid_depth = 0
        
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            # Get RGB frame
            in_rgb = q_rgb.tryGet()
            
            # Get depth data
            in_depth = q_depth.tryGet()
            
            # Process RGB frame if available
            if in_rgb is not None:
                # Get RGB frame
                rgb_frame = in_rgb.getCvFrame()
                
                try:
                    # Convert to ROS Image message
                    ros_image = bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
                    
                    # Add timestamp
                    ros_image.header.stamp = rospy.Time.now()
                    ros_image.header.frame_id = "oak_camera"
                    
                    # Publish frames
                    frames_pub.publish(ros_image)
                    
                except Exception as e:
                    rospy.logerr(f"Error converting image: {e}")
            
            # Process depth data if available
            if in_depth is not None:
                depth_frame = in_depth.getFrame()
                
                # Get center coordinates
                h, w = depth_frame.shape
                center_x, center_y = w // 2, h // 2
                
                # Get depth at center
                center_depth = depth_frame[center_y, center_x]
                
                # If center point is invalid, check a small window
                if center_depth < min_depth:
                    small_region = depth_frame[center_y-1:center_y+2, center_x-1:center_x+2]
                    valid_depths = small_region[small_region >= min_depth]
                    if valid_depths.size > 0:
                        center_depth = np.median(valid_depths)
                
                # Calculate depth in cm if valid
                if center_depth >= min_depth:
                    center_depth_cm = center_depth / 10.0
                    last_valid_depth = center_depth_cm  # Store the valid depth
                    
                    # Publish
                    depth_pub.publish(Float32(center_depth_cm))
                else:
                    # If no valid depth, publish last valid depth if available
                    if last_valid_depth > 0:
                        depth_pub.publish(Float32(last_valid_depth))
                    else:
                        # If no valid depth available yet, publish a negative value to indicate "too close"
                        depth_pub.publish(Float32(-1.0))
            
            # Sleep to maintain rate
            rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
