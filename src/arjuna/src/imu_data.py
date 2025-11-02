#!/usr/bin/env python3 
from time import sleep
import time
import board
import busio
import adafruit_bno055
import pandas as pd
import rospy
from std_msgs.msg import Float64,Int64
i2c = busio.I2C(board.SCL, board.SDA)
#i2c = busio.I2C(5, 3)
print(board.SCL)
print(board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
rotation_threshold = 3
euler_angles = sensor.euler
initial_heading = euler_angles[0]
rotation_status = 0
imu_data = 0
pub = rospy.Publisher('gravity_x', Float64, queue_size=10)
pub1 = rospy.Publisher('gravity_y', Float64, queue_size=10)
pub2 = rospy.Publisher('gravity_z', Float64, queue_size=10)
pub3 = rospy.Publisher('heading', Float64, queue_size=10)
pub4 = rospy.Publisher('roll', Float64, queue_size=10)
pub5 = rospy.Publisher('pitch', Float64, queue_size=10)
pub6 = rospy.Publisher('accleration_x', Float64, queue_size=10)
pub7 = rospy.Publisher('accleration_y', Float64, queue_size=10)
pub8 = rospy.Publisher('accleration_z', Float64, queue_size=10)
pub9 = rospy.Publisher('linear_accleration_x', Float64, queue_size=10)
pub10 = rospy.Publisher('linear_accleration_y', Float64, queue_size=10)
pub11 = rospy.Publisher('linear_accleration_z', Float64, queue_size=10)
pub12 = rospy.Publisher('gyro_x', Float64, queue_size=10)
pub13 = rospy.Publisher('gyro_y', Float64, queue_size=10)
pub14 = rospy.Publisher('gyro_z', Float64, queue_size=10)
#pub15 = rospy.Publisher('dxml_control_from_imu', Int64, queue_size=10)
pub15 = rospy.Publisher('imu_dxml_control', Int64, queue_size=10)

#pub15 = rospy.Publisher('dxml_control', Int64, queue_size=10)

while True:
	rospy.init_node('talkere', anonymous=True)
	accel_x,accel_y,accel_z = sensor.acceleration
	acceleration_magnitude = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
	


	
	quaternion = sensor.quaternion
	euler_angles = sensor.euler
	gravity = sensor.gravity
	linear_accleration = sensor.linear_acceleration
	#time.sleep(1)
	gyro = sensor.gyro
	gr_x = gravity[0]
	gr_y = gravity[1]
	gr_z = gravity[2]
	heading = euler_angles[0]
	pitch = euler_angles[1]
	roll = euler_angles[2]
	lin_acc_x = linear_accleration[0]
	lin_acc_y = linear_accleration[1]
	lin_acc_z = linear_accleration[2]
	gyro_x = gyro[0]
	gyro_y = gyro[1]
	gyro_z = gyro[2]
	
	#rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	pub.publish(gr_x)
	pub1.publish(gr_y)
	pub2.publish(gr_z)
	pub3.publish(heading)
	pub4.publish(roll)
	pub5.publish(pitch)
	pub6.publish(accel_x)
	pub7.publish(accel_y)
	pub8.publish(accel_z)
	pub9.publish(lin_acc_x)
	pub10.publish(lin_acc_y)
	pub11.publish(lin_acc_z)
	pub12.publish(gyro_x)
	pub13.publish(gyro_y)
	pub14.publish(gyro_z)
	heading_change = abs(abs(heading) - abs(initial_heading))
	rate.sleep()
	print("Accleration_Magnitude",acceleration_magnitude)
	if  acceleration_magnitude > 6.0 and acceleration_magnitude < 13.5:
		print("Robot is stationary.")
		pub15.publish(222)
	elif acceleration_magnitude > 13.5 or acceleration_magnitude < 6.0:
		print("Shift to hop")
		pub15.publish(111)
		
	#	time.sleep(3)
	#print("heading_change",heading_change)
	#time.sleep(1)
'''
	if heading_change > rotation_threshold:
		rotation_status = 1
		print("Rotation")
	else:
		rotation_status = 0
	initial_heading = heading
	print("Accleration_magnitude",acceleration_magnitude)
	if acceleration_magnitude > 8.0 and acceleration_magnitude < 10.0:
		pub15.publish(00)

	elif acceleration_magnitude > 11.0 or acceleration_magnitude < 9.0 and rotation_status == 1:
		pub15.publish(1)

	print(heading)
	print(gr_x)
	print(gr_y)
	print(gr_z)
	print(heading)
	print(roll)
	print(pitch)
	print(accel_x)
	print(accel_y)
	print(accel_z)
	print(lin_acc_x)
	print(lin_acc_y)
	print(lin_acc_z)
	print("gyro_x",gyro_x)
	print("gyro_y",gyro_y)
	print("gyro_z",gyro_z)

	rate.sleep()
'''
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

