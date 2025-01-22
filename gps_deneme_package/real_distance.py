import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from geopy.distance import geodesic
import math
from gps_deneme_package.helper.common import *

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')

        # Subscribers
        self.subscriber_1 = self.create_subscription(
            NavSatFix,
            IRIS_NAVSAT_TOPIC_NAME,
            self.navsat_callback_1,
            10
        )
        self.subscriber_2 = self.create_subscription(
            NavSatFix,
            IRIS_1_NAVSAT_TOPIC_NAME,
            self.navsat_callback_2,
            10
        )

        # Publisher
        self.distance_publisher = self.create_publisher(Float32MultiArray, SENSOR_DISTANCE_TOPIC_NAME, 10)

        # To store synchronized messages
        self.navsat_1_data = None
        self.navsat_2_data = None

    def navsat_callback_1(self, msg):
        self.navsat_1_data = msg
        self.check_and_calculate_distance()

    def navsat_callback_2(self, msg):
        self.navsat_2_data = msg
        self.check_and_calculate_distance()

    def check_and_calculate_distance(self):
        if self.navsat_1_data and self.navsat_2_data:
            # Check if the timestamps are the same
            if (self.navsat_1_data.header.stamp.sec == self.navsat_2_data.header.stamp.sec and
                self.navsat_1_data.header.stamp.nanosec == self.navsat_2_data.header.stamp.nanosec):
                
                # Extract latitude, longitude, and altitude
                coord_1 = (self.navsat_1_data.latitude, self.navsat_1_data.longitude)
                coord_2 = (self.navsat_2_data.latitude, self.navsat_2_data.longitude)
                alt_1 = self.navsat_1_data.altitude
                alt_2 = self.navsat_2_data.altitude

                # Calculate the horizontal distance using geopy
                horizontal_distance = geodesic(coord_1, coord_2).meters

                # Calculate the vertical (altitude) difference
                vertical_distance = abs(alt_1 - alt_2)

                # Compute the absolute 3D distance
                absolute_distance = math.sqrt(horizontal_distance**2 + vertical_distance**2)

                # Create the Float32MultiArray message
                msg = Float32MultiArray()
                msg.data = [
                    float(self.navsat_1_data.header.stamp.sec),       # sec
                    float(self.navsat_1_data.header.stamp.nanosec),   # nanosec
                    absolute_distance                                  # distance
                ]
                self.distance_publisher.publish(msg)

                # Log the distance and coordinates
                # self.get_logger().info(
                #     f"Absolute Distance: {absolute_distance:.2f} meters | "
                #     f"Horizontal: {horizontal_distance:.2f} m | Vertical: {vertical_distance:.2f} m"
                # )

                # Reset stored data
                self.navsat_1_data = None
                self.navsat_2_data = None


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
