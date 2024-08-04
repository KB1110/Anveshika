import time
import board
import adafruit_bno055
import asyncio
import websockets
import json

# Initialize I2C connection and BNO055 sensor
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

# For debugging, check if the sensor is detected
if not sensor:
    raise RuntimeError("Failed to initialize BNO055 sensor. Please check your connection.")

# Function to handle temperature anomalies
last_val = 0xFFFF
def temperature():
    global last_val
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

# Asynchronous function to send sensor data
async def send_sensor_data(websocket, path):
    while True:
        try:
            # Create a dictionary of sensor data
            data = {
                "temperature": sensor.temperature,
                "accelerometer": sensor.acceleration,
                "magnetometer": sensor.magnetic,
                "gyroscope": sensor.gyro,
                "euler_angle": sensor.euler,
                "quaternion": sensor.quaternion,
                "linear_acceleration": sensor.linear_acceleration,
                "gravity": sensor.gravity
            }

            # Convert data to JSON string
            data_json = json.dumps(data)
            
            # Send data over WebSocket
            await websocket.send(data_json)
            
            # Print the data to the console for debugging
            print("Sent data: ", data_json)

        except Exception as e:
            pass

        # Sleep for a second before sending the next set of data
        await asyncio.sleep(1)

# Main function to start the WebSocket server
async def main():
    try:
        print("Starting WebSocket server on ws://localhost:8765...")
        async with websockets.serve(send_sensor_data, "0.0.0.0", 8765):
            await asyncio.Future()  # run forever
    except Exception as e:
        print(f"Failed to start WebSocket server: {e}")

if __name__ == "__main__":
    asyncio.run(main())
