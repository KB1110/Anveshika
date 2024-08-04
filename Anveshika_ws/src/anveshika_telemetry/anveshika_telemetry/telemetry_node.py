import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
import threading
import time
import cv2

from sensor_msgs.msg import Image

from flask import Flask,  Response

app = Flask(__name__)

img = None

class Telemetry(Node):
    
    def __init__(self):
        super().__init__('telemetry_node')
        self.subscription = self.create_subscription(Image, '/map_plot', self.map_plot_callback, qos_profile_system_default)
        self.bridgeObject = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.map_plot_cv_image = None

    def map_plot_callback(self, msg):
        global img
        
        self.map_plot_cv_image = self.bridgeObject.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        ret, buffer = cv2.imencode(".jpg", self.map_plot_cv_image)

        img = buffer.tobytes()

    def timer_callback(self):
        pass

@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    global img
    while True:
        if img is not None:
            # print(img)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')
        time.sleep(0.1)  # Adjust this delay as needed to control FPS

def main(args=None):
    rclpy.init(args=args)

    telemetry_node = Telemetry()
    
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8088, debug=False, use_reloader=False))
    flask_thread.start()

    
    try:
        rclpy.spin(telemetry_node)

    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        flask_thread.join()
        telemetry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()