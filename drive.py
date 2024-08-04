import csv
import time
from adafruit_servokit import ServoKit
import gpiod

kit = ServoKit(channels=16)

#Servo
# 0 - Front Left Servo
# 1 - Front Right Servo
# 2 - Back Left Servo
# 3 - Back Right Servo
kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[1].set_pulse_width_range(500, 2500)
kit.servo[2].set_pulse_width_range(500, 2500)
kit.servo[3].set_pulse_width_range(500, 2500)

#Drive Motors
# 4 - Front Left
# 5 - Front Right
# 6 - Back left
# 7 - Back Right
kit.servo[4].set_pulse_width_range(1000, 2000)
kit.servo[5].set_pulse_width_range(1000, 2000)
kit.servo[6].set_pulse_width_range(1000, 2000)
kit.servo[7].set_pulse_width_range(1000, 2000)

kit.servo[0].angle = 90
kit.servo[1].angle = 90
kit.servo[2].angle = 90
kit.servo[3].angle = 90

kit.servo[4].angle = 90
kit.servo[5].angle = 90
kit.servo[6].angle = 90
kit.servo[7].angle = 90

time.sleep(1)

def read_latest_csv_values(file_path):
    """Continuously read the latest data from a CSV file."""

    fl_steer = None
    fr_steer = None
    rl_steer = None
    rr_steer = None
    fl_drive = None
    fr_drive = None
    rl_drive = None
    rr_drive = None

    try:
        while True:
            # Open the CSV file in read mode
            with open(file_path, mode='r') as csv_file:
                # Create a CSV reader object
                csv_reader = csv.DictReader(csv_file)

                # Get the last row from the CSV file
                for row in csv_reader:

                    fl_steer = float(row['fl_steer'])
                    fr_steer = float(row['fr_steer'])
                    rl_steer = float(row['rl_steer'])
                    rr_steer = float(row['rr_steer'])
                    fl_drive = float(row['fl_drive'])
                    fr_drive = float(row['fr_drive'])
                    rl_drive = float(row['rl_drive'])
                    rr_drive = float(row['rr_drive'])

                kit.servo[0].angle = fl_steer
                kit.servo[1].angle = fr_steer
                kit.servo[2].angle = rl_steer
                kit.servo[3].angle = rr_steer

                kit.servo[4].angle = fl_drive
                kit.servo[5].angle = fr_drive
                kit.servo[6].angle = rl_drive
                kit.servo[7].angle = rr_drive

            # Wait for a short period before reading again
            time.sleep(0.5)  # Sleep for 1 second (adjust as needed)

    except FileNotFoundError:
        # print(drive, front_steer, back_steer)
        print(f"File not found: {file_path}")
    except KeyboardInterrupt:
        # print(drive, front_steer, back_steer)
        print("Stopping the data reading loop.")
    except Exception as e:
        # print(drive, front_steer, back_steer)
        print(f"An error occurred: {e}")

    finally:
        # print(drive, front_steer, back_steer)
        kit.servo[0].angle = 90
        kit.servo[1].angle = 90
        kit.servo[2].angle = 90
        kit.servo[3].angle = 90
        
        kit.servo[4].angle = 90
        kit.servo[5].angle = 90
        kit.servo[6].angle = 90
        kit.servo[7].angle = 90

if __name__ == "__main__":
    # Specify the path to your CSV file
    file_path = '/home/pi/Anveshika/driving_data.csv'
    read_latest_csv_values(file_path)
