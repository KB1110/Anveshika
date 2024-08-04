import csv
import time
from adafruit_servokit import ServoKit
import gpiod

kit = ServoKit(channels=16)

# Arm Servo
# 8  - Base
# 9  - Shoulder
# 10 - Elbow
# 11 - Wrist 1
# 12 - Wrist 2
# 13 - Gripper

kit.servo[8].set_pulse_width_range(500, 2500)
kit.servo[9].set_pulse_width_range(500, 2500)
kit.servo[10].set_pulse_width_range(500, 2500)
kit.servo[11].set_pulse_width_range(500, 2500)
kit.servo[12].set_pulse_width_range(500, 2500)
kit.servo[13].set_pulse_width_range(500, 2500)

servo_default_angles = [90, 120, 180, 90, 120, 90]

base = servo_default_angles[0]
shoulder = servo_default_angles[1]
elbow = servo_default_angles[2]
wrist1 = servo_default_angles[3]
wrist2 = servo_default_angles[4]
gripper = servo_default_angles[5]

kit.servo[8].angle = base 
kit.servo[9].angle = shoulder 
kit.servo[10].angle = elbow 
kit.servo[11].angle = wrist1 
kit.servo[12].angle = wrist2 
kit.servo[13].angle = gripper

prev_base = base 
prev_shoulder = shoulder
prev_elbow = elbow 
prev_wrist1 = wrist1 
prev_wrist2 = wrist2 
prev_gripper = gripper

time.sleep(1)

def move_servo(servo_num, prev_angle, target_angle):
    try:
        while abs(target_angle - prev_angle) >= 2:
            print(servo_num, target_angle, prev_angle, abs(target_angle - prev_angle))
            if target_angle > prev_angle:
                prev_angle += 2
                kit.servo[servo_num].angle = prev_angle
                time.sleep(0.05)
            else:
                prev_angle -= 2
                kit.servo[servo_num].angle = prev_angle
                time.sleep(0.05)

        kit.servo[servo_num].angle = target_angle
    except:
        pass

def read_latest_csv_values(file_path):
    """Continuously read the latest data from a CSV file."""

    global prev_base
    global prev_shoulder
    global prev_elbow
    global prev_wrist1
    global prev_wrist2
    global prev_gripper

    global base
    global shoulder
    global elbow
    global wrist1
    global wrist2
    global gripper

    try:
        while True:
            # Open the CSV file in read mode
            with open(file_path, mode='r') as csv_file:
                # Create a CSV reader object

                print("HELLO")

                csv_reader = csv.DictReader(csv_file)

                # Get the last row from the CSV file
                for row in csv_reader:
                    base = float(row["base"])
                    shoulder = float(row["shoulder"])
                    elbow = float(row["elbow"])
                    wrist1 = float(row["wrist1"])
                    wrist2 = float(row["wrist2"])
                    gripper = float(row["gripper"])


                move_servo(12, prev_wrist2, wrist2)
                move_servo(8, prev_base, base)
                move_servo(9, prev_shoulder, shoulder)
                move_servo(10, prev_elbow, elbow)
                move_servo(11, prev_wrist1, wrist1)
                move_servo(13, prev_gripper, gripper)
                
                kit.servo[8].angle = base
                kit.servo[9].angle = shoulder
                kit.servo[10].angle = elbow
                kit.servo[11].angle = wrist1
                kit.servo[12].angle = wrist2
                kit.servo[13].angle = gripper

                prev_base = base
                prev_shoulder = shoulder
                prev_elbow = elbow
                prev_wrist1 = wrist1
                prev_wrist2 = wrist2
                prev_gripper = gripper

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
        move_servo(8, prev_base, base)
        move_servo(9, prev_shoulder, shoulder)
        move_servo(10, prev_elbow, elbow)
        move_servo(11, prev_wrist1, wrist1)
        move_servo(12, prev_wrist2, wrist2)
        move_servo(13, prev_gripper, gripper)

if __name__ == "__main__":
    # Specify the path to your CSV file
    file_path = '/home/pi/Anveshika/arm_data.csv'
    read_latest_csv_values(file_path)
