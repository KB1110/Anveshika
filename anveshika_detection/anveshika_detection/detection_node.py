# import cv2
# import rclpy
# from sensor_msgs.msg import Image
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from rclpy.qos import qos_profile_system_default

# import numpy as np

# lower_red1 = np.array([0, 100, 100])
# upper_red1 = np.array([10, 255, 255])

# lower_red2 = np.array([160, 100, 100])
# upper_red2 = np.array([180, 255, 255])

# lower_red = np.array([0, 50, 35])
# upper_red = np.array([10, 255, 255])

# class DetectionNode(Node):
#     def __init__(self):
#         super().__init__('detection_node')
#         self.bridgeObject = CvBridge()

#         self.colorImageTopic = '/camera/camera/color/image_raw'
#         self.depthImageTopic = '/camera/camera/depth/image_rect_raw'

#         self.colorSubscriber = self.create_subscription(Image, self.colorImageTopic, self.colorCallback, qos_profile_system_default)
#         self.depthSubscriber = self.create_subscription(Image, self.depthImageTopic, self.depthCallback, qos_profile_system_default)

#         self.image_pub = self.create_publisher(Image, '/map_plot', qos_profile_system_default)

#         self.timer_ = self.create_timer(0.1, self.detectionCallback)

#         self.colorOpenCVImage = None
#         self.depthOpenCVImage = None

#     def colorCallback(self, imageMessage):
#         self.get_logger().info("The color frame is received")
#         self.colorOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
#         # cv2.imshow("Color", self.colorOpenCVImage)
#         cv2.waitKey(1)

#     def depthCallback(self, imageMessage):
#         self.get_logger().info("The depth frame is received")
#         # self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
#         self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='passthrough')
#         # cv2.imshow("Depth", self.depthOpenCVImage)
#         cv2.waitKey(1)

#     def detectionCallback(self):
#         if self.colorOpenCVImage is not None and self.depthOpenCVImage is not None:

            # depth_colormap = cv2.applyColorMap(self.depthOpenCVImage, cv2.COLORMAP_JET)
            # edge = cv2.Canny(depth_colormap, 500, 700)
#             left_border_width = 30  # Adjust this value as needed

#             # Mask out the left border region
#             edge[:, :left_border_width] = 0

#             # Find contours
#             contours, hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             # Draw contours
#             contour_image = np.copy(depth_colormap)

#             for i, contour in enumerate(contours):
#                 if cv2.contourArea(contour) > 40:  # Filter out small contours
#                     # Get bounding box for each contour
#                     x, y, w, h = cv2.boundingRect(contour)
                    
#                     # Calculate the distance for the center point of the contour
#                     center_x, center_y = x + w // 2, y + h // 2
#                     # distance = depth_frame.get_distance(center_x, center_y)

#                     distance = map_value_to_mm(self.depthOpenCVImage[center_y, center_x][0])
#                     print(distance)

#                     # Draw contour and bounding box
#                     cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 2)
#                     cv2.rectangle(contour_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

#                     # Annotate distance
#                     cv2.putText(contour_image, f'Object: {distance/1000:.2f}m', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                     # cv2.putText(contour_image, f'Object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


#             img_msg = self.bridgeObject.cv2_to_imgmsg(contour_image)

#             self.image_pub.publish(img_msg)
#             # Show images
            # cv2.imshow('Canny Edge', edge)
            # cv2.imshow('Detected Objects', contour_image)

            # Convert to HSV
            # hsv = cv2.cvtColor(self.colorOpenCVImage, cv2.COLOR_BGR2HSV)

            # # Create masks for red color
            # # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            # # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

            # # mask = mask1 | mask2

            # mask = cv2.inRange(hsv, lower_red, upper_red)

            # # Find contours
            # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # for contour in contours:
            #     # Filter contours by area
            #     area = cv2.contourArea(contour)

            #     if area > 500:
            #         # Approximate contour to a polygon
            #         epsilon = 0.02 * cv2.arcLength(contour, True)
            #         approx = cv2.approxPolyDP(contour, epsilon, True)

            #         # Calculate contour perimeter and circularity
            #         perimeter = cv2.arcLength(approx, True)

            #         if perimeter > 0:
            #             circularity = 4 * np.pi * area / (perimeter * perimeter)

            #             # Define a range for circularity

            #             if 0.7 < circularity < 1.3:
            #                 # Draw bounding rectangle and contour
            #                 x, y, w, h = cv2.boundingRect(contour)
            #                 cv2.rectangle(self.colorOpenCVImage, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #                 cv2.drawContours(self.colorOpenCVImage, [approx], 0, (0, 255, 0), 2)
            #                 cv2.putText(self.colorOpenCVImage, "Red Cylinder", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            #         # Fit an ellipse to the contour
            #         if len(contour) >= 5: # Ellipse fitting requires at least 5 points

            #             ellipse = cv2.fitEllipse(contour)
            #             (x, y), (MA, ma), angle = ellipse
            #             aspect_ratio = ma / MA

            #             # Define aspect ratio range for cylindrical shapes
            #             if 0.7 < aspect_ratio < 1.3:

            #                 # Draw ellipse
            #                 cv2.ellipse(self.colorOpenCVImage, ellipse, (0, 0, 255), 2)
            #                 cv2.putText(self.colorOpenCVImage, "Red Cylinder", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # # Display the result

            # cv2.imshow('Video Feed', self.colorOpenCVImage)
            # cv2.imshow('Mask', mask)

# def map_value_to_mm(value):
#     """
#     Map an integer value between 0 and 255 to a measurement in millimeters,
#     where 0 maps to 100 mm and 255 maps to 1500 mm.

#     Parameters:
#     value (int): An integer value between 0 and 255.

#     Returns:
#     float: The corresponding measurement in millimeters.
#     """
#     # Ensure value is within the correct range
#     if not (0 <= value <= 255):
#         raise ValueError("Input value must be between 0 and 255.")

#     # Define the meter range
#     min_meters = 0.1
#     max_meters = 2.5

#     # Calculate the corresponding meter value
#     meter_value = min_meters + (value / 255) * (max_meters - min_meters)

#     # Convert meters to millimeters
#     millimeter_value = meter_value * 1000

#     return millimeter_value

# def main(args=None):
#     rclpy.init(args=args)

#     detection_node = DetectionNode()

#     rclpy.spin(detection_node)

#     detection_node.destroy_node()

#     rclpy.shutdown()

# if __name__=="__main__":
#     main()

import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
import math
from std_msgs.msg import Float32
import time
from std_msgs.msg import String

import numpy as np

# TO DO
# Make a error, error_x and error_y publisher
# Use error in navigation node to drive the robot in search and travel modes
# Use error_x and error_y in sample pick and drop modes with the arm_node

lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([5, 255, 255])

lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

centered_start_time = None
centered_start_time_x = None
centered_start_time_y = None

# lower_red = np.array([0, 50, 35])
# upper_red = np.array([10, 255, 255])

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridgeObject = CvBridge()

        self.colorImageTopic = '/camera/camera/color/image_raw'
        self.depthImageTopic = '/camera/camera/depth/image_rect_raw'

        self.colorSubscriber = self.create_subscription(Image, self.colorImageTopic, self.colorCallback, qos_profile_system_default)
        self.depthSubscriber = self.create_subscription(Image, self.depthImageTopic, self.depthCallback, qos_profile_system_default)

        self.mode_publisher = self.create_publisher(String, '/mode_in', qos_profile_system_default)
        self.mode_subscriber = self.create_subscription(String, '/mode_out', self.mode_callback, qos_profile_system_default)

        self.error_x_publisher = self.create_publisher(Float32, '/error_x', qos_profile_system_default)
        self.error_y_publisher = self.create_publisher(Float32, '/error_y', qos_profile_system_default)
        
        self.centered_x_publisher = self.create_publisher(Float32, '/center_x', qos_profile_system_default)
        self.centered_y_publisher = self.create_publisher(Float32, '/center_y', qos_profile_system_default)

        self.mode = "4"

        self.centered_x = False
        self.centered_y = False

        self.depth_publisher = self.create_publisher(Image, "/depth_image", qos_profile_system_default)

        self.image_publisher = self.create_publisher(Image, '/map_plot', qos_profile_system_default)

        self.timer_ = self.create_timer(0.1, self.detectionCallback)

        self.colorOpenCVImage = None
        self.depthOpenCVImage = None

    def colorCallback(self, imageMessage):
        # self.get_logger().info("The color frame is received")
        self.colorOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
        # cv2.imshow("Color", self.colorOpenCVImage)
        cv2.waitKey(1)

    def depthCallback(self, imageMessage):
        # self.get_logger().info("The depth frame is received")
        # self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='bgr8')
        self.depthOpenCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage, desired_encoding='passthrough')
        # cv2.imshow("Depth", self.depthOpenCVImage)
        cv2.waitKey(1)

    def mode_callback(self, msg):
        self.mode = msg.data

    def calculate_real_life_dimensions(pixel_width, pixel_height, real_life_depth, focal_length):
        # Define the pixel to cm ratio (based on previous calculations)
        pixel_to_cm_ratio_width = 7.5   # Width: 7.5 pixels/cm
        pixel_to_cm_ratio_height = 6.67 # Height: 6.67 pixels/cm

        # Calculate real-life dimensions considering depth
        real_life_width = (pixel_width * real_life_depth) / (focal_length * pixel_to_cm_ratio_width)
        real_life_height = (pixel_height * real_life_depth) / (focal_length * pixel_to_cm_ratio_height)

        return real_life_width, real_life_height

    def detectionCallback(self):
        global centered_start_time
        if self.colorOpenCVImage is not None and self.depthOpenCVImage is not None:

            depth_colormap = cv2.applyColorMap(self.depthOpenCVImage, cv2.COLORMAP_JET)
            edge = cv2.Canny(depth_colormap, 5, 10000)
            left_border_width = 30  # Adjust this value as needed

            depth_image = cv2.cvtColor(self.depthOpenCVImage, cv2.COLOR_RGB2GRAY)

            depth_image = self.bridgeObject.cv2_to_imgmsg(depth_image)

            color_colormap = np.copy(self.colorOpenCVImage)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_colormap.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                color_colormap = cv2.resize(color_colormap, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            
            # image = np.hstack((color_colormap, depth_colormap))

            # ros_image = self.bridgeObject.cv2_to_imgmsg(image)

            # self.image_publisher.publish(ros_image)

            #Search for sample and travel to sample
            if self.mode == "2" or self.mode == "3":

                hsv = cv2.cvtColor(color_colormap, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (15, 15), 0)
                
                #Red Sample Detection
                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = mask1 | mask2

                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 300:
                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.intp(box)

                        print(box)

                        cv2.drawContours(color_colormap, [box], 0, (0, 255, 0), 2)

                        center_x = (box[0][0] + box[2][0]) / 2
                        center_y = (box[0][1] + box[2][1]) / 2
                        
                        # Get depth at the center of the contour
                        distance = map_value_to_mm(self.depthOpenCVImage[int(center_y), int(center_x)][0])
                        print(distance)

                        x, y = int(rect[0][0]), int(rect[0][1])

                        focal_length = 280
                        pixel_to_cm_ratio_width = 9.8   # Width: 7.5 pixels/cm
                        # pixel_to_cm_ratio_height = 6.67 # Height: 6.67 pixels/cm
                        pixel_to_cm_ratio_height = 9.8

                        w1 = abs(box[0][0] - box[1][0])
                        w2 = abs(box[0][0] - box[3][0])

                        w = max(w1, w2)

                        h1 = abs(box[1][1] - box[2][1])
                        h2 = abs(box[0][1] - box[2][1])

                        h = max(h1, h2)

                        real_life_width = (w * distance) / (focal_length * pixel_to_cm_ratio_width)
                        real_life_height = (h * distance) / (focal_length * pixel_to_cm_ratio_height)

                        # # Calculate the horizontal angle based on the camera's field of view

                        cv2.putText(color_colormap, f'Red Sample d: {(distance / 1000):.2f}m', (x - 30, y - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_colormap, f'w: {(real_life_width):.2f}cm h: {(real_life_height):.2f}cm', (x - 30, y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Map the center_x to a servo angle (0 to 180 degrees)
                        frame_center_x = color_colormap.shape[1] // 2
                        error_x = frame_center_x - center_x

                        err_x = Float32()
                        err_x.data = error_x

                        print(err_x.data, error_x)

                        self.error_x_publisher.publish(err_x)

                        center_x_msg = Float32()
                        center_x_msg.data = 0.0
                        self.centered_x_publisher.publish(center_x_msg)
                    
                        # Object is centered
                        if abs(error_x) <= 50:  # Adjust the tolerance as needed
                            if centered_start_time is None:
                                centered_start_time = time.time()
                            elif time.time() - centered_start_time > 0.2:
                                # object is confirmed to be centered
                                print("Centered")
                                cv2.putText(color_colormap, f'Centered', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                center_x_msg.data = 1.0
                                self.centered_x_publisher.publish(center_x_msg)
                                # break  # End the program
                        else:
                            centered_start_time = None

                    # Correct orientation logic
                # cv2.imshow('Video Feed', color_colormap)
                # cv2.imshow('Mask', mask)

            #Search for container and travel to container
            elif self.mode == "5" or self.mode == "6":

                hsv = cv2.cvtColor(color_colormap, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (15, 15), 0)
                
                #Red Sample Detection
                mask = cv2.inRange(hsv, lower_blue, upper_blue)

                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 300:
                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.intp(box)

                        print(box)

                        cv2.drawContours(color_colormap, [box], 0, (0, 255, 0), 2)

                        center_x = (box[0][0] + box[2][0]) / 2
                        center_y = (box[0][1] + box[2][1]) / 2
                        
                        # Get depth at the center of the contour
                        distance = map_value_to_mm(self.depthOpenCVImage[int(center_y), int(center_x)][0])
                        print(distance)

                        x, y = int(rect[0][0]), int(rect[0][1])

                        focal_length = 280
                        pixel_to_cm_ratio_width = 9.8   # Width: 7.5 pixels/cm
                        # pixel_to_cm_ratio_height = 6.67 # Height: 6.67 pixels/cm
                        pixel_to_cm_ratio_height = 9.8

                        w1 = abs(box[0][0] - box[1][0])
                        w2 = abs(box[0][0] - box[3][0])

                        w = max(w1, w2)

                        h1 = abs(box[1][1] - box[2][1])
                        h2 = abs(box[0][1] - box[2][1])

                        h = max(h1, h2)

                        real_life_width = (w * distance) / (focal_length * pixel_to_cm_ratio_width)
                        real_life_height = (h * distance) / (focal_length * pixel_to_cm_ratio_height)

                        # # Calculate the horizontal angle based on the camera's field of view

                        cv2.putText(color_colormap, f'Blue Container d: {(distance / 1000):.2f}m', (x - 30, y - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_colormap, f'w: {(real_life_width):.2f}cm h: {(real_life_height):.2f}cm', (x - 30, y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Map the center_x to a servo angle (0 to 180 degrees)
                        frame_center_x = color_colormap.shape[1] // 2
                        error = frame_center_x - center_x
                        
                        # Object is centered
                        if abs(error) <= 50:  # Adjust the tolerance as needed
                            if centered_start_time is None:
                                centered_start_time = time.time()
                            elif time.time() - centered_start_time > 0.2:
                                # object is confirmed to be centered
                                print("Centered")
                                cv2.putText(color_colormap, f'Centered', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                # break  # End the program
                        else:
                            centered_start_time = None

                    # Correct orientation logic
                # cv2.imshow('Video Feed', color_colormap)
                # cv2.imshow('Mask', mask)
        
            #Sample Pickup
            if self.mode == "4":

                global centered_start_time_x
                global centered_start_time_y

                hsv = cv2.cvtColor(color_colormap, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (15, 15), 0)
                
                #Red Sample Detection
                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = mask1 | mask2

                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 300:
                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.intp(box)

                        # print(box)

                        cv2.drawContours(color_colormap, [box], 0, (0, 255, 0), 2)

                        center_x = (box[0][0] + box[2][0]) / 2
                        center_y = (box[0][1] + box[2][1]) / 2
                        
                        # Get depth at the center of the contour
                        distance = map_value_to_mm(self.depthOpenCVImage[int(center_y), int(center_x)][0])
                        # print(distance)

                        x, y = int(rect[0][0]), int(rect[0][1])

                        focal_length = 280
                        pixel_to_cm_ratio_width = 9.8   # Width: 7.5 pixels/cm
                        # pixel_to_cm_ratio_height = 6.67 # Height: 6.67 pixels/cm
                        pixel_to_cm_ratio_height = 9.8

                        w1 = abs(box[0][0] - box[1][0])
                        w2 = abs(box[0][0] - box[3][0])

                        w = max(w1, w2)

                        h1 = abs(box[1][1] - box[2][1])
                        h2 = abs(box[0][1] - box[2][1])

                        h = max(h1, h2)

                        real_life_width = (w * distance) / (focal_length * pixel_to_cm_ratio_width)
                        real_life_height = (h * distance) / (focal_length * pixel_to_cm_ratio_height)

                        cv2.putText(color_colormap, f'Red Sample d: {(distance / 1000):.2f}m', (x - 30, y - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(color_colormap, f'w: {(real_life_width):.2f}cm h: {(real_life_height):.2f}cm', (x - 30, y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        if not self.centered_x:
                            # Map the center_x to a servo angle (0 to 180 degrees)
                            frame_center_x = color_colormap.shape[1] // 2
                            error_x = frame_center_x - center_x
                            err_x = Float32()
                            err_x.data = error_x

                            print(err_x.data, error_x)

                            self.error_x_publisher.publish(err_x)

                            center_x_msg = Float32()
                            center_x_msg.data = 0.0
                            self.centered_x_publisher.publish(center_x_msg)
                            
                            # Object is centered
                            if abs(error_x) <= 30:  # Adjust the tolerance as needed
                                if centered_start_time_x is None:
                                    centered_start_time_x = time.time()
                                elif time.time() - centered_start_time_x > 1.0:
                                    # object is confirmed to be centered
                                    print("Centered")
                                    cv2.putText(color_colormap, f'w: {(real_life_width):.2f}cm h: {(real_life_height):.2f}cm', (x - 30, y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                    self.centered_x = True
                                    center_x_msg.data = 1.0
                                    self.centered_x_publisher.publish(center_x_msg)
                            else:
                                centered_start_time_x = None

                        if self.centered_x and not self.centered_y:
                            # Map the center_x to a servo angle (0 to 180 degrees)
                            frame_center_y = color_colormap.shape[0] // 2
                            error_y = frame_center_y - center_y

                            err_y = Float32()
                            err_y.data = error_y

                            self.error_x_publisher.publish(err_y)
                            center_y_msg = Float32()
                            center_y_msg.data = 0.0
                            self.centered_y_publisher.publish(center_y_msg)
                            
                            # Object is centered
                            if abs(error_y) <= 30:  # Adjust the tolerance as needed
                                if centered_start_time_y is None:
                                    centered_start_time_y = time.time()
                                elif time.time() - centered_start_time_y > 0.1:
                                    # object is confirmed to be centered
                                    print("Centered")
                                    self.centered_y = True
                                    center_y_msg.data = 1.0
                                    self.centered_y_publisher.publish(center_y_msg)
                                    
                            else:
                                centered_start_time_y = None

                    # Correct orientation logic
                # cv2.imshow('Video Feed', color_colormap)
                # cv2.imshow('Mask', mask)

                # image = np.hstack((color_colormap, mask))

                ros_image = self.bridgeObject.cv2_to_imgmsg(color_colormap)

                self.image_publisher.publish(ros_image)

# ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file ~/Anveshika/Anveshika_ws/install/anveshika_detection/share/anveshika_detection/params/realsense.yaml

def map_value_to_mm(value):
    """
    Map an integer value between 0 and 255 to a measurement in millimeters,
    where 0 maps to 100 mm and 255 maps to 1500 mm.

    Parameters:
    value (int): An integer value between 0 and 255.

    Returns:
    float: The corresponding measurement in millimeters.
    """
    # Ensure value is within the correct range
    if not (0 <= value <= 255):
        raise ValueError("Input value must be between 0 and 255.")

    # Define the meter range
    min_meters = 0.1
    max_meters = 2.5

    # Calculate the corresponding meter value
    meter_value = min_meters + (value / 255) * (max_meters - min_meters)

    # Convert meters to millimeters
    millimeter_value = meter_value * 1000

    return millimeter_value


def main(args=None):
    rclpy.init(args=args)

    detection_node = DetectionNode()

    rclpy.spin(detection_node)

    detection_node.destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    main()