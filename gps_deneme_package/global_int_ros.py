import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped  # Import GeoPoseStamped
from pymavlink import mavutil
from gps_deneme_package.helper.common import *
from gps_deneme_package.helper.mavlink_functions_req import *

class MavlinkToROS2Node(Node):
    def __init__(self):
        super().__init__('mavlink_to_ros2')
        
        self.publisher_ = self.create_publisher(GeoPoseStamped, IRIS_GLOBAL_POSITION_INT, 10)
        
        self.timer = self.create_timer(0.04, self.timer_callback)  # 10 Hz
        self.timer_1 = self.create_timer(0.5, self.timer_1_callback)    
        
        self.mavlink_connection = mavutil.mavlink_connection(IRIS_CONNECTION_STRING_1)
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info('Connected to MAVLink on udp:127.0.0.1:14550')

    def timer_1_callback(self):
        send_command(self.mavlink_connection, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 1, 33, 50000)

    def timer_callback(self):
        message = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

        if message:
            geopose_msg = GeoPoseStamped()
            geopose_msg.header.stamp = self.get_clock().now().to_msg()
            geopose_msg.header.frame_id = 'map'
            
            geopose_msg.pose.position.latitude = message.lat / 1e7  # Decimal degrees
            geopose_msg.pose.position.longitude = message.lon / 1e7
            geopose_msg.pose.position.altitude = message.alt / 1000.0  # Meters
            
            geopose_msg.pose.orientation.x = 0.0
            geopose_msg.pose.orientation.y = 0.0
            geopose_msg.pose.orientation.z = 0.0
            geopose_msg.pose.orientation.w = 1.0
            
            self.publisher_.publish(geopose_msg)
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
