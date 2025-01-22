import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import pandas as pd
import sys
class NavSatFixToCSV(Node):
    def __init__(self, topic_name, output_file):
        super().__init__('navsatfix_to_csv')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.write_to_file = False

        # Parameters
        self.topic_name = topic_name
        self.output_file = output_file
        self.sec = None
        self.nanosec = None
        
        # Initialize list to store data
        self.nav_data = []

        # Check if output directory exists
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Subscriber
        self.subscription = self.create_subscription(
            NavSatFix,
            self.topic_name,
            self.navsatfix_callback,
            self.qos_profile
        )
        self.get_logger().info(f"Subscribed to topic: {self.topic_name}")

    def navsatfix_callback(self, msg):
        try:
            sec = msg.header.stamp.sec
            nanosec = f"{int(msg.header.stamp.nanosec):09d}"

            if not (self.sec == sec and self.nanosec == nanosec) and self.sec is not None:
                # Store data in list
                self.nav_data.append({
                    'Sec': sec,
                    'Nanosec': nanosec,
                    'Latitude': msg.latitude,
                    'Longitude': msg.longitude,
                    'Altitude': msg.altitude
                })
                
                self.get_logger().info(f"Stored NavSatFix data: Sec={sec}, Nanosec={nanosec}")
                
            elif self.sec is None:
                self.sec = sec
                self.nanosec = nanosec
                self.nav_data.append({
                    'Sec': sec,
                    'Nanosec': nanosec,
                    'Latitude': msg.latitude,
                    'Longitude': msg.longitude,
                    'Altitude': msg.altitude
                })
            else:
                self.get_logger().info(f"Duplicate Sec and Nanosec - Saving sorted data...")
                self.save_sorted_data()
                super().destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to store NavSatFix data: {e}")

    def save_sorted_data(self):
        try:
            df = pd.DataFrame(self.nav_data)
            df_sorted = df.sort_values(by=['Sec', 'Nanosec'])
            df_sorted.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Sorted CSV file saved at: {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save sorted data: {e}")

    def destroy_node(self):
        if self.nav_data:
            self.save_sorted_data()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: ros2 run <package_name> <node_name> <topic_name> <output_file>")
        sys.exit(1)
    topic_name = sys.argv[1]
    output_file = sys.argv[2]
    node = NavSatFixToCSV(topic_name, output_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()