import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys
import pandas as pd

class PoseToCSV(Node):
    def __init__(self, pose_topic, output_file):
        super().__init__('pose_to_csv')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.write_to_file = False

        # Parameters
        self.pose_topic = pose_topic
        self.output_file = output_file
        self.sec = None
        self.nanosec = None
        
        # Initialize list to store data
        self.point_data = []

        # Check if output directory exists
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Subscriber
        self.subscription = self.create_subscription(
            PointStamped,
            self.pose_topic,
            self.pose_callback,
            self.qos_profile
        )
        self.get_logger().info(f"Subscribed to topic: {self.pose_topic}")
        self.get_logger().info(f"Output file: {self.output_file}")

    def pose_callback(self, msg):
        try:
            if not (self.sec == msg.header.stamp.sec and self.nanosec == msg.header.stamp.nanosec) and self.sec is not None:
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec

                # Store data in list
                self.point_data.append({
                    'Sec': sec,
                    'Nanosec': nanosec,
                    'Position X': msg.point.x,
                    'Position Y': msg.point.y,
                    'Position Z': msg.point.z
                })
                
                self.get_logger().info(f"Stored point data: Sec={sec}, Nanosec={nanosec}")
                
            elif self.sec is None:
                self.sec = msg.header.stamp.sec
                self.nanosec = msg.header.stamp.nanosec
                self.point_data.append({
                    'Sec': self.sec,
                    'Nanosec': self.nanosec,
                    'Position X': msg.point.x,
                    'Position Y': msg.point.y,
                    'Position Z': msg.point.z
                })
            else:
                self.get_logger().info(f"Duplicate Sec and Nanosec - Saving sorted data...")
                self.save_sorted_data()
                super().destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to store point data: {e}")

    def save_sorted_data(self):
        try:
            df = pd.DataFrame(self.point_data)
            df_sorted = df.sort_values(by=['Sec', 'Nanosec'])
            df_sorted.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Sorted CSV file saved at: {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save sorted data: {e}")

    def destroy_node(self):
        if self.point_data:
            self.save_sorted_data()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: ros2 run <package_name> <node_name> <pose_topic> <output_file>")
        sys.exit(1)
    pose_topic = sys.argv[1]
    output_file = sys.argv[2]
    node = PoseToCSV(pose_topic, output_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()