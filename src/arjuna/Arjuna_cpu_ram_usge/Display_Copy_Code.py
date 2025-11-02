import Adafruit_SSD1306
import time
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import psutil
import smbus2  # Import smbus2 for I2C communication

# I2C channel (bus 0 for reading sensor, bus 1 for OLED display)
bus_read = smbus2.SMBus(0)  # Use bus 0 for reading sensor values
bus_display = smbus2.SMBus(1)  # Use bus 1 for OLED display

# Arduino Nano I2C address
arduino_address = 0x08

# Initialize the OLED display (on bus 1)
OLED = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)
OLED.begin()
OLED.clear()
OLED.display()

# Load the default font for display
font = ImageFont.load_default()
width = OLED.width
height = OLED.height

# Create a blank image for drawing
image = Image.new("1", (width, height))

# Function to read sensor values from bus 0
def read_sensor_values():
    try:
        # Read 4 bytes of data from Arduino (2 bytes for each analog value)
        data = bus_read.read_i2c_block_data(arduino_address, 0, 4)
        sensorValueA0 = (data[0] << 8) | data[1]
        sensorValueA1 = (data[2] << 8) | data[3]
        print(sensorValueA0)
        print(sensorValueA1)
        return sensorValueA0, sensorValueA1  # Return values for further use
    except OSError as e:
        print("I2C Error: {}".format(e))
        return None, None  # Return None if there's an error

while True:
    # Read the sensor values from Arduino
    sensorValueA0, sensorValueA1 = read_sensor_values()

    # Get system stats
    cpu_usage = psutil.cpu_percent()
    ram_usage = psutil.virtual_memory().percent

    # Calculate voltages based on sensor values
    Voltage_Charger = sensorValueA0 * 10.60 / 430
    Voltage_Battery = sensorValueA1 * 11.47 / 466
    Voltage_Battery = round(Voltage_Battery, 2)

    # Create a drawing context
    draw = ImageDraw.Draw(image)

    # Clear the image
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Draw the text on the image
    draw.text((42, 0), "ARJUNA", font=font, fill=255)
    draw.text((0, 10), "CPU:{}%".format(cpu_usage), font=font, fill=255)
    draw.text((70, 10), "RAM:{}%".format(ram_usage), font=font, fill=255)
    draw.text((20, 20), "Voltage:{} V".format(Voltage_Battery), font=font, fill=255)

    # Display the image on the OLED
    OLED.image(image)
    OLED.display()

    time.sleep(5)

    # Check if Charging (if sensor A0 value is greater than A1)
    if sensorValueA0 > sensorValueA1:
        draw = ImageDraw.Draw(image)

        # Clear the image
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        # Draw the text on the image
        draw.text((42, 0), "ARJUNA", font=font, fill=255)
        draw.text((0, 10), "Voltage:{}V".format(Voltage_Battery), font=font, fill=255)
        draw.text((0, 20), "Charging...", font=font, fill=255)

        # Display the image on the OLED
        OLED.image(image)
        OLED.display()

        time.sleep(5)
