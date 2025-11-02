#! /usr/bin/env python
import rospy
import Adafruit_SSD1306
import smbus2
from std_msgs.msg import Float64

# I2C channel (usually 1 on Jetson Nano)
bus = smbus2.SMBus(1)
# Arduino Nano I2C address
arduino_address = 0x08

def read_sensor_values():
    try:
        # Read 4 bytes of data from Arduino (2 bytes for each analog value)
        data = bus.read_i2c_block_data(arduino_address, 0, 4)
        sensorValueA1 = (data[2] << 8) | data[3]
        return sensorValueA1  # Return values for further use
    except OSError as e:
        print("I2C Error: {}".format(e))
        return None, None  # Return None if there's an error

def process():
    rospy.init_node("Voltage_to_webpage" , anonymous = True)
    voltage_pub = rospy.Publisher("/VOLTAGE" , Float64 , queue_size = 10)
    while True:
        sensorValueA1 = read_sensor_values()
        Voltage_Battery = sensorValueA1 * 11.43 / 470
        Voltage_Battery = round(Voltage_Battery,2)
        voltage_pub.publish(Voltage_Battery)
        print(Voltage_Battery)

process()

        

