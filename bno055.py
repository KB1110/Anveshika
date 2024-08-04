import time
import board
import adafruit_bno055
import csv

# Initialize the I2C connection to the sensor
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

last_val = 0xFFFF

def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

# Open the CSV file in write mode
with open('/home/pi/Anveshika/imu_readings.csv', 'w', newline='') as csvfile:
    # Create a CSV writer
    csv_writer = csv.writer(csvfile)
    
    # Write the header row
    csv_writer.writerow(["Temperature (C)", "Accelerometer (m/s^2)", 
                         "Magnetometer (microteslas)", "Gyroscope (rad/sec)",
                         "Euler angle", "Quaternion", "Linear acceleration (m/s^2)", 
                         "Gravity (m/s^2)"])
    
    while True:
        # Get sensor readings
        temperature = sensor.temperature
        acceleration = sensor.acceleration
        magnetic = sensor.magnetic
        gyro = sensor.gyro
        euler = sensor.euler
        quaternion = sensor.quaternion
        linear_acceleration = sensor.linear_acceleration
        gravity = sensor.gravity

        # Print the readings to the console
        print("Temperature: {} degrees C".format(temperature))
        print("Accelerometer (m/s^2): {}".format(acceleration))
        print("Magnetometer (microteslas): {}".format(magnetic))
        print("Gyroscope (rad/sec): {}".format(gyro))
        print("Euler angle: {}".format(euler))
        print("Quaternion: {}".format(quaternion))
        print("Linear acceleration (m/s^2): {}".format(linear_acceleration))
        print("Gravity (m/s^2): {}".format(gravity))
        print()
        
        # Move the file pointer to the beginning of the file (after the header row)
        csvfile.seek(0)
        
        # Write the sensor readings to the CSV file
        csv_writer.writerow([temperature, acceleration, magnetic, gyro, euler, 
                             quaternion, linear_acceleration, gravity])

        # Flush the file to ensure data is written
        csvfile.flush()

        # Wait for 1 second before reading again
        time.sleep(1)
