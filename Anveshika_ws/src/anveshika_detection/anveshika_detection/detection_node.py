import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default

import numpy as np

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridgeObject = CvBridge()

        self.colorImageTopic = '/camera/camera/color/image_raw'
        self.depthImageTopic = '/camera/camera/depth/image_rect_raw'

        self.colorSubscriber = self.create_subscription(Image, self.colorImageTopic, self.colorCallback, qos_profile_system_default)
        self.depthSubscriber = self.create_subscription(Image, self.depthImageTopic, self.depthCallback, qos_profile_system_default)

        self.timer_ = self.create_timer(0.1, self.detectionCallback)

        self.colorOpenCVImage = None
        self.depthOpenCVImage = None

    def colorCallback(self, imageMessage):
        self.get_logger().info("The color frame is received")
        self.colorOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
        cv2.imshow("Color", self.colorOpenCVImage)
        cv2.waitKey(1)

    def depthCallback(self, imageMessage):
        self.get_logger().info("The depth frame is received")
        # self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
        self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='passthrough')
        cv2.imshow("Depth", self.depthOpenCVImage)
        cv2.waitKey(1)

    def detectionCallback(self):
        if self.colorOpenCVImage is not None and self.depthOpenCVImage is not None:
            depth_colormap = cv2.convertScaleAbs(self.depthOpenCVImage, alpha=0.03)
            # cv2.imshow('DC1', depth_colormap)

            depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
            # cv2.imshow('DC2', depth_colormap)

            edge = cv2.Canny(depth_colormap, 100, 500)

            left_border_width = 30  # Adjust this value as needed

            # Mask out the left border region
            edge[:, :left_border_width] = 0

            # Find contours
            contours, hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw contours
            contour_image = np.copy(depth_colormap)

            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) > 12:  # Filter out small contours
                    # Get bounding box for each contour
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate the distance for the center point of the contour
                    center_x, center_y = x + w // 2, y + h // 2
                    # distance = depth_frame.get_distance(center_x, center_y)

                    distance = self.depthOpenCVImage[center_y, center_x]
                    print(distance)

                    # Draw contour and bounding box
                    cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 2)
                    cv2.rectangle(contour_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

                    # Annotate distance
                    cv2.putText(contour_image, f'Object: {distance/1000:.2f}m', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(contour_image, f'Object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show images
            cv2.imshow('Canny Edge', edge)
            cv2.imshow('Detected Objects', contour_image)

def main(args=None):
    rclpy.init(args=args)

    detection_node = DetectionNode()

    rclpy.spin(detection_node)

    detection_node.destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()