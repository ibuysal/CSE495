import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
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
        self.pose_data = []

        # Check if the output directory exists, create if not
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            self.qos_profile
        )
        self.get_logger().info(f"Subscribed to topic: {self.pose_topic}")
        self.get_logger().info(f"Output file: {self.output_file}")

    def pose_callback(self, msg):
        """Callback to process and save PoseStamped messages."""
        try:
            # Extract timestamp
            if not (self.sec == msg.header.stamp.sec and self.nanosec == msg.header.stamp.nanosec) and self.sec is not None:
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec

                # Extract position
                position = msg.pose.position

                # Store data in list
                self.pose_data.append({
                    'Sec': sec,
                    'Nanosec': nanosec,
                    'Position X': position.x,
                    'Position Y': position.y,
                    'Position Z': position.z
                })
                
                self.get_logger().info(f"Stored pose data: Sec={sec}, Nanosec={nanosec}, "
                                     f"Position=({position.x}, {position.y}, {position.z})")
                
            elif self.sec is None:
                self.sec = msg.header.stamp.sec
                self.nanosec = msg.header.stamp.nanosec
                self.pose_data.append({
                    'Sec': self.sec,
                    'Nanosec': self.nanosec,
                    'Position X': msg.pose.position.x,
                    'Position Y': msg.pose.position.y,
                    'Position Z': msg.pose.position.z
                })
            else:
                self.get_logger().info(f"Duplicate Sec and Nanosec - Saving sorted data...")
                self.save_sorted_data()
                super().destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to store pose data: {e}")

    def save_sorted_data(self):
        """Convert stored data to DataFrame, sort, and save to CSV."""
        try:
            # Convert to DataFrame
            df = pd.DataFrame(self.pose_data)
            
            # Sort by Sec and Nanosec
            df_sorted = df.sort_values(by=['Sec', 'Nanosec'])
            
            # Save to CSV
            df_sorted.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Sorted CSV file saved at: {self.output_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save sorted data: {e}")

    def destroy_node(self):
        """Save sorted data and close node."""
        if self.pose_data:
            self.save_sorted_data()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Parse arguments
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