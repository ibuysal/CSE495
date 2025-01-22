import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
import csv
import os
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import pandas as pd

class GeoPoseToCSV(Node):
    def __init__(self, pose_topic, output_file):
        super().__init__('geopose_to_csv')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # Parameters
        self.pose_topic = pose_topic
        self.output_file = output_file
        self.sec = None
        self.nanosec = None
        self.write_to_file = False
        
        # Initialize list to store data
        self.pose_data = []

        # Check if output directory exists
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Subscriber
        self.subscription = self.create_subscription(
            GeoPoseStamped,
            self.pose_topic,
            self.pose_callback,
            self.qos_profile
        )
        self.get_logger().info(f"Subscribed to topic: {self.pose_topic}")
        self.get_logger().info(f"Will save data to: {self.output_file}")

    def pose_callback(self, msg):
        try:
            if not (self.sec == msg.header.stamp.sec and self.nanosec == msg.header.stamp.nanosec) and self.sec is not None:
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec
                position = msg.pose.position

                # Store data in list
                self.pose_data.append({
                    'Sec': sec,
                    'Nanosec': nanosec,
                    'Latitude': position.latitude,
                    'Longitude': position.longitude,
                    'Altitude': position.altitude
                })
                
                self.get_logger().info(f"Stored GeoPose data: Sec={sec}, Nanosec={nanosec}")
                
            elif self.sec is None:
                self.sec = msg.header.stamp.sec
                self.nanosec = msg.header.stamp.nanosec
                self.pose_data.append({
                    'Sec': self.sec,
                    'Nanosec': self.nanosec,
                    'Latitude': msg.pose.position.latitude,
                    'Longitude': msg.pose.position.longitude,
                    'Altitude': msg.pose.position.altitude
                })
            else:
                self.get_logger().info(f"Duplicate Sec and Nanosec - Saving sorted data...")
                self.save_sorted_data()
                super().destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to store GeoPose data: {e}")

    def save_sorted_data(self):
        try:
            df = pd.DataFrame(self.pose_data)
            df_sorted = df.sort_values(by=['Sec', 'Nanosec'])
            df_sorted.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Sorted CSV file saved at: {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save sorted data: {e}")

    def destroy_node(self):
        if self.pose_data:
            self.save_sorted_data()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: ros2 run <package_name> <node_name> <pose_topic> <output_file>")
        sys.exit(1)
    pose_topic = sys.argv[1]
    output_file = sys.argv[2]
    node = GeoPoseToCSV(pose_topic, output_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()