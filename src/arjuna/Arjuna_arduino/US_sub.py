#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class UltrasonicSubscriber:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ultrasonic_subscriber', anonymous=True)
        
        # Subscribers for sensor data
        self.array_sub = rospy.Subscriber('/ultrasonic_distances', Float32MultiArray, self.array_callback)
        self.point_sub = rospy.Subscriber('/ultrasonic_point', Point, self.point_callback)
        
        rospy.loginfo("Ultrasonic Subscriber Node Started")
        
    def array_callback(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 2:
            sensor1_distance = msg.data[0]
            sensor2_distance = msg.data[1]
            
            print("[Array] Sensor 1: "+str(sensor1_distance)+" cm | Sensor 2: "+str(sensor2_distance)+" cm")
            
            # Add distance-based alerts
            if sensor1_distance < 10:
                print("  -> WARNING: Sensor 1 detects close obstacle!")
            if sensor2_distance < 10:
                print("  -> WARNING: Sensor 2 detects close obstacle!")
                
    def point_callback(self, msg):
        """Callback for Point messages"""
        sensor1_distance = msg.x
        sensor2_distance = msg.y
        
        print("[Point] Sensor 1: "+str(sensor1_distance)+" cm | Sensor 2: "+str(sensor2_distance)+" cm")
        
        # Calculate average distance
        avg_distance = (sensor1_distance + sensor2_distance) / 2
        print("  -> Average distance: "+str(avg_distance)+" cm")
        
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = UltrasonicSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
