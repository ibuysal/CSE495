# # scripts/script_module.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from gps_deneme_package.helper.common import *


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            IRIS_IMAGE_TOPIC_NAME, 
            self.image_callback,
            10
        )
        self.subscription  
        self.bridge = CvBridge()  
        self.get_logger().info("Image Subscriber node initialized.")

        self.bb_center_pub = self.create_publisher(PointStamped, IRIS_IMAGE_RECTANGLE_CENTER_TOPIC_NAME, 10)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            processed_image = self.detect_red_drone(cv_image)

            cv2.imshow("Drone Detection", processed_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def detect_red_drone(self, image):
        """
        Detect the red target drone in the image using a red mask.
        :param image: Input image (OpenCV format).
        :return: Processed image with the drone highlighted.
        """
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the red color range in HSV
        lower1 = np.array([0, 19, 0])
        upper1 = np.array([19,255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        lower2 = np.array([156,19,0])
        upper2 = np.array([180,255,255])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        
        combined_mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((1,1), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        max_area = 0
        largest_box = None
        
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
      
            if area > max_area:
                max_area = area
                largest_box = (x, y, w, h)
        
        blank_image=np.zeros_like(image)
        if largest_box is not None:
            x, y, w, h = largest_box
            cx=int(x+w/2)
            cy=y
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
            self.publish_bounding_box_center(cx, cy)

        return image

    def publish_bounding_box_center(self, cx, cy):
        """
        Publishes the center of the bounding box as a PointStamped message.
        :param cx: Center x-coordinate of the bounding box (pixels).
        :param cy: Center y-coordinate of the bounding box (pixels).
        """
        center_msg = PointStamped()
        center_msg.header.stamp = self.get_clock().now().to_msg()
        center_msg.header.frame_id = "camera_frame" 
        center_msg.point.x = float(cx)
        center_msg.point.y = float(cy)
        center_msg.point.z = 0.0 
        self.bb_center_pub.publish(center_msg)
        # self.get_logger().info(f"Published Bounding Box Center: cx={cx}, cy={cy}")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    # Close down the node and any OpenCV windows
    image_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from gps_deneme_package.helper.common import *
# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         # Define the topic name and type
#         self.subscription = self.create_subscription(
#             Image,
#             IRIS_IMAGE_TOPIC_NAME,  # Replace with your actual image topic name
#             self.image_callback,
#             10
#         )
#         self.subscription  # Prevent unused variable warning
#         self.bridge = CvBridge()  # Bridge to convert ROS images to OpenCV format
#         self.get_logger().info("Image Subscriber node initialized.")

#     def image_callback(self, msg):
#         try:
#             # Convert the ROS Image message to an OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             self.get_logger().info("Image received.")

#             # Process the image to detect the red target drone
#             processed_image = self.detect_red_drone(cv_image)

#             # Optionally display the processed image (comment out for headless systems)
#             cv2.imshow("Drone Detection", processed_image)
#             cv2.waitKey(1)

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")

#     def detect_red_drone(self, image):
#         """
#         Detect the red target drone in the image using a red mask.
#         :param image: Input image (OpenCV format).
#         :return: Processed image with the drone highlighted.
#         """
#         # Convert the image to HSV color space
#         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#         # Define the red color range in HSV
#         lower_red1 = np.array([0, 120, 70])  # Adjust as necessary
#         upper_red1 = np.array([10, 255, 255])
#         lower_red2 = np.array([170, 120, 70])  # Adjust as necessary
#         upper_red2 = np.array([180, 255, 255])

#         # Create masks for the red color
#         mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#         mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#         red_mask = cv2.bitwise_or(mask1, mask2)
#         cv2.imshow("mask",red_mask)

#         # Find contours on the red mask
#         contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # Draw rectangles around the detected red regions (assume largest is the drone)
#         for contour in contours:
#             if cv2.contourArea(contour) > 5:  # Minimum area threshold to avoid noise
#                 x, y, w, h = cv2.boundingRect(contour)
#                 cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 self.get_logger().info(f"Red drone detected at: x={x}, y={y}, w={w}, h={h}")

#         return image
# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = ImageSubscriber()
#     rclpy.spin(image_subscriber)

#     # Close down the node and any OpenCV windows
#     image_subscriber.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

