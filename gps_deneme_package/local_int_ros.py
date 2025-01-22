import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # Import GeoPoseStamped
from pymavlink import mavutil
from gps_deneme_package.helper.common import *
from gps_deneme_package.helper.mavlink_functions_req import *

class MavlinkToROS2Node(Node):
    def __init__(self):
        super().__init__('mavlink_to_ros2_2')
        
        # Create a publisher for the ROS 2 topic
        self.publisher_ = self.create_publisher(PoseStamped, IRIS_LOCAL_POSITION_INT, 10)
        
        # Timer to publish messages periodically
        self.timer = self.create_timer(0.04, self.timer_callback)  # 10 Hz
        self.timer_1 = self.create_timer(0.5, self.timer_1_callback)    
        
        # MAVLink connection
        self.mavlink_connection = mavutil.mavlink_connection(IRIS_CONNECTION_STRING_4)
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info('Connected to MAVLink on udp:127.0.0.1:14550')

    def timer_1_callback(self):
        # Send MAVLink command to set the message interval
        send_command(self.mavlink_connection, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 1, 32, 50000)

    def timer_callback(self):
        # Wait for a MAVLink message
        message = self.mavlink_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)

        if message:
            # Create a GeoPoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = message.x
            pose_msg.pose.position.y = message.y
            pose_msg.pose.position.z = message.z
            
            # Publish to the ROS 2 topic
            self.publisher_.publish(pose_msg)
            # self.get_logger().info(f'Published GeoPose: lat={geopose_msg.pose.position.latitude}, lon={geopose_msg.pose.position.longitude}, alt={geopose_msg.pose.position.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkToROS2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
