import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np
import sys

class ImuSubscriber(Node):
    def __init__(self, imu_topic='/imu', rpy_topic='/rpy'):
        super().__init__('imu_subscriber')

        # Subscriber for IMU data
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic}")

        # Publisher for roll, pitch, yaw
        self.publisher = self.create_publisher(Float32MultiArray, rpy_topic, 10)
        self.get_logger().info(f"Publishing RPY to topic: {rpy_topic}")

    def quaternion_to_euler(self, x, y, z, w):
        """
        Converts quaternion (x, y, z, w) to roll, pitch, yaw.
        """
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        """
        Callback function to process incoming IMU data and publish RPY.
        """
        # Extract quaternion from IMU message
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Compute roll, pitch, and yaw
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        # Create Float32MultiArray message for roll, pitch, yaw
        rpy_msg = Float32MultiArray()
        rpy_msg.data = [roll_deg, pitch_deg, yaw_deg]  # RPY in radians

        # Publish the RPY message
        self.publisher.publish(rpy_msg)

        # Log the RPY values
        # self.get_logger().info(
        #     f"Published RPY - Roll: {np.degrees(roll):.2f}°, Pitch: {np.degrees(pitch):.2f}°, Yaw: {np.degrees(yaw):.2f}°"
        # )

def main(args=None):
    rclpy.init(args=args)

    # Default topic names
    imu_topic = '/imu'
    rpy_topic = '/rpy'

    # Read custom topic names from command-line arguments
    if len(sys.argv) > 1:
        imu_topic = sys.argv[1]
    if len(sys.argv) > 2:
        rpy_topic = sys.argv[2]

    node = ImuSubscriber(imu_topic, rpy_topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
