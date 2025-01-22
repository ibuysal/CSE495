import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time
from gps_deneme_package.helper.common import *
from gps_deneme_package.helper.mavlink_functions_req import *

class DualDroneController(Node):
    def __init__(self):
        super().__init__('dual_drone_controller')

        # ROS 2 Subscribers
        self.takeoff_signal_subscriber = self.create_subscription(
            Bool,
            TAKEOFF_SIGNAL,
            self.takeoff_callback,
            10
        )
        self.xy_coordinates_subscriber = self.create_subscription(
            Point,
            IRIS_1_XY,
            self.xy_coordinates_callback,
            10
        )
        self.xy_coordinates_subscriber = self.create_subscription(
            Point,
            IRIS_XY,
            self.xy0_coordinates_callback,
            10
        )

        self.iris = None
        self.iris1 = None
        self.xy_coordinates = None

        self.connect_to_drones()

    def connect_to_drones(self):
        """Connect to both drones using pymavlink."""
        try:
            self.get_logger().info("Connecting to Iris...")
            self.iris = mavutil.mavlink_connection(IRIS_CONNECTION_STRING_3)
            self.iris.wait_heartbeat()
            self.get_logger().info("Iris connected!")

            self.get_logger().info("Connecting to Iris1...")
            self.iris1 = mavutil.mavlink_connection(IRIS_1_CONNECTION_STRING_2)
            self.iris1.wait_heartbeat()
            self.get_logger().info("Iris1 connected!")
        except Exception as e:
            self.get_logger().error(f"Error connecting to drones: {e}")

    def takeoff_callback(self, msg):
        """Callback to handle takeoff signal."""
        if msg.data:
            self.get_logger().info("Takeoff signal received. Taking off drones.")
            if self.iris and self.iris1:
                self.arm_and_takeoff(self.iris, 20.0)
                self.arm_and_takeoff(self.iris1, 20.0)
            else:
                self.get_logger().error("One or both drones are not connected.")
        else:
            self.get_logger().info("Takeoff signal is False. Ignoring.")

    def arm_and_takeoff(self, drone, altitude):
        """Arm and take off the drone."""
        self.get_logger().info("Checking if drone is armable...")
        set_mode_send(drone, "GUIDED")
        send_command(drone,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,1,77,50000)

        while True:
            heartbeat = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and heartbeat.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                break
            self.get_logger().info("Waiting for drone to become armable... Ensure pre-arm checks are passed.")
            time.sleep(1)

        self.get_logger().info("Drone is armable. Arming...")
        while True:
            drone.mav.command_long_send(
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            ack = self.wait_for_command_ack(drone, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack:
                self.get_logger().info("Drone armed successfully.")
                break
            self.get_logger().warning("Arming failed. Retrying...")
            time.sleep(1)

        while True:
            heartbeat = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                break
            self.get_logger().info("Waiting for arming...")
            time.sleep(1)

        self.get_logger().info(f"Drone armed. Taking off to {altitude} meters...")

        while True:
            takeoff(drone,altitude)

            ack = self.wait_for_command_ack(drone, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            if ack:
                self.get_logger().info("Takeoff command accepted.")
                break
            self.get_logger().warning("Takeoff command not acknowledged. Retrying...")
            time.sleep(1)

        send_command(drone,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,1,33,50000)

        while True:
            altitude_msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if altitude_msg and altitude_msg.relative_alt >= altitude * 1000 * 0.95:
                self.get_logger().info("Reached target altitude.")
                break
            time.sleep(1)
    def wait_for_command_ack(self, drone, command):
        """Wait for a command acknowledgment."""
        start_time = time.time()
        while time.time() - start_time < 5:  # 5-second timeout
            msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == command and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return True
        return False
    def xy_coordinates_callback(self, msg):
        """Callback to handle x, y coordinates for Iris."""
        if self.iris:
            self.xy_coordinates = (msg.x, msg.y)
            self.get_logger().info(f"Received coordinates: x={msg.x}, y={msg.y}. Moving Iris.")
            self.send_local_ned(self.iris1, msg.x, msg.y, -20.0)  # Maintain altitude at 10 meters
    def xy0_coordinates_callback(self, msg):
        """Callback to handle x, y coordinates for Iris."""
        if self.iris:
            self.xy_coordinates = (msg.x, msg.y)
            self.get_logger().info(f"Received coordinates: x={msg.x}, y={msg.y}. Moving Iris.")
            self.send_local_ned(self.iris, msg.x, msg.y, -20.0)  # Maintain altitude at 10 meters

    def send_local_ned(self, drone, x, y, z):
        """Send a local NED position to a drone."""
        try:
            ignore_accel = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE| mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE| mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
            self.get_logger().info(f"Sending local NED position: x={x}, y={y}, z={z}")
            drone.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                drone.target_system,  # Target system
                drone.target_component,  # Target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame (MAV_FRAME_LOCAL_NED)
                ignore_accel,  # Type mask (ignore velocities and accelerations)
                round(x, 3),  # X position
                round(y, 3),  # Y position
                round(z, 3),  # Z position
                0, 0, 0,  # Velocities (ignored)
                0, 0, 0,  # Accelerations (ignored)
                0,  # Yaw (ignored)
                0   # Yaw rate (ignored)
            )
        except Exception as e:
            self.get_logger().error(f"Error sending local NED command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DualDroneController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
