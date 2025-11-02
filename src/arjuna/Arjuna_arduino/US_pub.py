#!/usr/bin/env python

import rospy
import serial
import json
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class UltrasonicPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ultrasonic_publisher', anonymous=True)
        
        # Publisher for sensor data
        self.sensor_pub = rospy.Publisher('/ultrasonic_distances', Float32MultiArray, queue_size=10)
        
        # Alternative publisher using Point message (x=sensor1, y=sensor2)
        self.point_pub = rospy.Publisher('/ultrasonic_point', Point, queue_size=10)
        
        # Serial connection parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo("Connected to Arduino on " + str(self.serial_port))
        except Exception as e:
            rospy.logerr("Failed to connect to Arduino: " + str(e))
            return
        
        
    def run(self):
        # Continuous infinite loop to read and publish data
        while not rospy.is_shutdown():
            try:
                # Continuously read from Arduino serial port
                line = self.arduino.readline().decode('utf-8').strip()
                
                if line:
                    # Parse JSON data
                    data = json.loads(line)
                    sensor1_distance = data.get('sensor1', 0)
                    sensor2_distance = data.get('sensor2', 0)
                    
                    # Create Float32MultiArray message
                    array_msg = Float32MultiArray()
                    array_msg.data = [sensor1_distance, sensor2_distance]
                    
                    # Create Point message
                    point_msg = Point()
                    point_msg.x = sensor1_distance
                    point_msg.y = sensor2_distance
                    point_msg.z = 0.0
                    
                    # Publish messages immediately
                    self.sensor_pub.publish(array_msg)
                    self.point_pub.publish(point_msg)
                    
                    rospy.loginfo("Published: Sensor1=" + str(sensor1_distance) + "cm, Sensor2=" + str(sensor2_distance) + "cm")
                    
            except json.JSONDecodeError:
                rospy.logwarn("Invalid JSON received: " + str(line))
            except Exception as e:
                rospy.logerr("Error reading from Arduino: " + str(e))
                break
                
            # No rate limiting - publish as fast as data arrives from Arduino
        
        # Close serial connection
        if hasattr(self, 'arduino'):
            self.arduino.close()

if __name__ == '__main__':
    try:
        publisher = UltrasonicPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
