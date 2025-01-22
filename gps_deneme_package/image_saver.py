import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys

class BagImageSaver(Node):
    def __init__(self, output_directory):
        super().__init__('bag_image_saver')

        # Parameters
        self.image_topic = '/camera/image'  # Topic to subscribe
        self.output_directory = output_directory  # Directory to save images
        self.image_format = 'jpg'  # Use 'png' or 'jpg'
        self.sec = None
        self.nanosec = None

        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10  # QoS depth
        )
        self.bridge = CvBridge()  # For converting ROS Image to OpenCV format
        self.get_logger().info(f"Subscribed to topic: {self.image_topic}")
        self.get_logger().info(f"Images will be saved to: {self.output_directory}")

    def image_callback(self, msg):
        """Callback to process and save images."""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec  # Format nanoseconds to 9 characters with leading zeros
            timestamp = f"{sec}_{nanosec}"

            if not (self.sec == sec and self.nanosec == nanosec) and self.sec is not None:
                # Format nanoseconds to 9 characters
                # Write data to CSV
                if self.write_to_file:
                    filename = os.path.join(self.output_directory, f"{timestamp}.{self.image_format}")
                # Save the image
                    cv2.imwrite(filename, cv_image)
                    self.get_logger().info(f"Saved image: {filename}")
            elif self.sec is None:
                self.sec = sec
                self.nanosec = nanosec
                self.write_to_file = True
                filename = os.path.join(self.output_directory, f"{timestamp}.{self.image_format}")
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Saved image: {filename}")

            else:
                self.get_logger().info(f"Duplicate Sec and Nanosec")
                self.write_to_file = False
                super().destroy_node()

                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Take output directory as an argument
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> <node_name> <output_directory>")
        sys.exit(1)

    output_directory = sys.argv[1]
    node = BagImageSaver(output_directory)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
