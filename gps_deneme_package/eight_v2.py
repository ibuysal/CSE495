import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil
import time
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gps_deneme_package.helper.common import *
from gps_deneme_package.helper.mavlink_functions_req import *
from std_msgs.msg import Bool
import threading
# Parameters for the eight trajectory

# Global variables to store current drone position and heading


from pyproj import Proj, transform
from pyproj import Transformer


# Rotation matrices
def rotation_matrix_x(angle_deg):
    """Create a rotation matrix for a rotation around the X-axis."""
    angle_rad = np.radians(angle_deg)
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle_rad), -np.sin(angle_rad)],
        [0, np.sin(angle_rad), np.cos(angle_rad)]
    ])
def rotation_matrix_y(angle_deg):
    """Create a rotation matrix for a rotation around the Y-axis."""
    angle_rad = np.radians(angle_deg)
    return np.array([
        [np.cos(angle_rad), 0, np.sin(angle_rad)],
        [0, 1, 0],
        [-np.sin(angle_rad), 0, np.cos(angle_rad)]
    ])

def rotation_matrix_z(angle_deg):
    """Create a rotation matrix for a rotation around the Z-axis."""
    angle_rad = np.radians(angle_deg)
    return np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad), np.cos(angle_rad), 0],
        [0, 0, 1]
    ])

# Generate the eight trajectory in the body frame
def generate_eight_in_body_frame(amplitude, length, points):
    """Generate an eight-shaped trajectory in the body frame."""
    theta = np.linspace(0, 2 * np.pi, points)
    x = length * np.sin(theta)  # Longitudinal direction
    y = amplitude * np.cos(theta) * np.sin(theta)  # Lateral direction
    z = np.zeros_like(theta)  # Altitude is constant
    return np.vstack((x, y, z))  # Stack into a 3xN matrix

# Transform trajectory to the global frame
def transform_to_global(body_points, heading, position, angle_x, angle_y, angle_z):
    """Transform body frame points to the global frame."""
    # Heading rotation matrix (around Z-axis in the global frame)
    heading_rad = np.radians(heading)
    heading_rotation = np.array([
        [np.cos(heading_rad), -np.sin(heading_rad), 0],
        [np.sin(heading_rad), np.cos(heading_rad), 0],
        [0, 0, 1]
    ])

    # Rectangle inclination rotation matrix (around Y-axis in the body frame)
    inclination_rotation = rotation_matrix_y(angle_y)

    # Rectangle orientation rotation matrix (around Z-axis in the body frame)
    orientation_rotation = rotation_matrix_z(angle_z)

    # Add rotation around the X-axis
    x_axis_rotation = rotation_matrix_x(angle_x)

    # Combine rotations: Y-axis rotation applied after heading, then X and Z
    total_rotation = heading_rotation @ inclination_rotation @ x_axis_rotation @ orientation_rotation

    # Transform points
    global_points = total_rotation @ body_points

    # Translate points to the drone's position
    global_points[0, :] += position[0]
    global_points[1, :] += position[1]
    global_points[2, :] += position[2]

    return global_points

def visualize_trajectory(global_points, center, heading):
    """Visualize the eight trajectory and the drone's heading in 3D."""
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the trajectory in the global frame
    ax.plot(global_points[0, :], global_points[1, :], global_points[2, :],
            label="Eight Trajectory", color="b")

    ax.scatter(center[0], center[1], center[2], color="r", label="Drone Position", s=50)

    heading_rad = np.radians(heading)
    arrow_length = 10.0  # Length of the arrow
    dx = arrow_length * np.cos(heading_rad)
    dy = arrow_length * np.sin(heading_rad)
    dz = 0  
   
    ax.set_title("Eight Trajectory Visualization", fontsize=14)
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Altitude [m]")
    ax.legend()

    plt.show()
from geometry_msgs.msg import PointStamped

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('eight_trajectory_node')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE,     
            depth=10                                   
        )
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            self.qos_profile  
        )

        self.rpy_subscriber = self.create_subscription(
            Float32MultiArray, MOVEMENT_PARAMS, self.params_callback, 10
        )
        self.parameter_subscriber = self.create_subscription(
            Float32MultiArray, '/rpy_1', self.rpy_callback, 10
        )
        self.x_y_z_publisher = self.create_publisher(PointStamped, IRIS_1_TRAJECTORY_POINTS_TOPIC_NAME, 10)
        self.feature_publisher=self.create_publisher(Float32MultiArray, '/trajectory_features',10)
        self.go_signal_subscriber = self.create_subscription(
            Bool,
            IRIS_1_GO_SIGNAL,
            self.go_signal_callback,
            10
        )
        self.drone = self.connect_drone()
        # self.timer = self.create_timer(1.0, self.execute)  # Call execute every 1 second
        self.execution_in_progress = False
        
        self.amplitude = 3.0  # Half-width of the rectangle
        self.length = 3.0  # Half-length of the rectangle
        self.points = 250  # Number of points in the trajectory
        self.angel_x = 90
        self.angel_y = 0  # Inclination angle of the rectangle in degrees (around Y-axis)
        self.angle_z = 0  # Orientation angle of the rectangle in degrees (around Z-axis)
        self.update_interval = 0.5  # Time between sending trajectory points

        self.stop_event = threading.Event()  # Stop flag for the thread

        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.center_received = False
        self.current_heading = 0.0

        self.heading_received = False

    def pose_callback(self, msg):
        """Callback to update the drone's position."""
        if not self.center_received:
            self.current_x = round(msg.pose.position.x,3)#
            self.current_y = round(msg.pose.position.y,3)#
            self.current_z =round( msg.pose.position.z,2)#20#
            self.center_received = True
    def params_callback(self, msg):
        """Callback to update the drone's position."""
        data=msg.data
        self.amplitude = data[0]  # Half-width of the rectangle
        self.length = data[1]  # Half-length of the rectangle
        self.points = int(data[2])  # Number of points in the trajectory
        self.angel_x = int(data[3])
        self.angel_y = int(data[4])  # Inclination angle of the rectangle in degrees (around Y-axis)
        self.angle_z = int(data[5])  # Orientation angle of the rectangle in degrees (around Z-axis)
        self.update_interval = data[6]
        self.get_logger().info(f"Drone parameters setted!")
        self.get_logger().info(f"{msg.data}")



    def rpy_callback(self, msg):
        """Callback to update the drone's heading."""
        if not self.heading_received:
            if msg.data[2] < 0:
                self.current_heading = 360 + int(msg.data[2])
            else:
                self.current_heading = int(msg.data[2])
            self.heading_received = True

    def connect_drone(self):
        """Connect to the drone via MAVLink."""
        self.get_logger().info("Connecting to the drone...")
        drone = mavutil.mavlink_connection(IRIS_1_CONNECTION_STRING_4)
        drone.wait_heartbeat()
        self.get_logger().info("Drone connected!")
        return drone
    def extract_advanced_trajectory_features(self,trajectory):
        total_length = np.sum(np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1)))
        direct_dist = np.linalg.norm(trajectory[-1] - trajectory[0])
        
        if direct_dist < 1e-10:  # Small threshold to avoid division by zero
            sinuosity = 1.0  # or some default value
        else:
            sinuosity = total_length / direct_dist
        # Turning analysis
        vectors = np.diff(trajectory, axis=0)
        angles = np.arctan2(vectors[1:,1], vectors[1:,0]) - np.arctan2(vectors[:-1,1], vectors[:-1,0])
        mean_angle = np.mean(angles)
        angle_var = np.var(angles)

        # Spatial distribution
        centroid = np.mean(trajectory, axis=0)
        r_gyr = np.sqrt(np.mean(np.sum((trajectory - centroid)**2, axis=1)))
        bbox = np.max(trajectory, axis=0) - np.min(trajectory, axis=0)

        # Key points
        start = trajectory[0]
        end = trajectory[-1]
        mid = trajectory[len(trajectory)//2]
        height_var = np.max(trajectory[:,2]) - np.min(trajectory[:,2])

        features = np.concatenate([
            [total_length, direct_dist, sinuosity],  # 3
            [mean_angle, angle_var],                 # 2
            centroid,                                # 3
            [r_gyr],                                 # 1
            bbox,                                    # 3
            start, mid, end,                         # 9
            [height_var]                             # 1
        ])

        return features
    def publish_xyz(self,x,y,z):
        msg = PointStamped()

        # Fill in the header with the current timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Frame of reference (e.g., "map", "world", etc.)

        # Fill in the point data
        msg.point.x = x # Example x value
        msg.point.y = y  # Example y value
        msg.point.z = z  # Example z value

        self.x_y_z_publisher.publish(msg)
        # self.get_logger().info(
        #     f'Published: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
        #     f'x={msg.point.x}, y={msg.point.y}, z={msg.point.z}'
        # )
    def send_trajectory_to_drone(self,global_points,altitude_difference):
        ignore_velocity = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE|mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
             | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        ignore_accel = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE| mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE| mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )

        #set_yaw(self.drone,0)
        time.sleep(3)
        do_change_speed(self.drone,2)

        for i in range(global_points.shape[1]):
            if self.stop_event.is_set():
                break
            x=global_points[0, i]
            y=global_points[1, i]
            z=global_points[2, i]
            self.drone.mav.set_position_target_local_ned_send(
                0,#time_pair[1] + int(round((time.time() - time_pair[0]) * 1000)),
                1,
                1,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                # (ignore_velocity |  ),
                ignore_accel,
                round(x,3),
                round(y,3),
                round(-z,4),  # Altitude
                0, 0, 0,  # Velocities
                0,0,0,  # Accelerations
                self.current_heading,  # Yaw
                0  # Yaw rate
            )
            self.publish_xyz(y,x,z)
            print(f"point {i+1} x {y},y {x} z {round(-z,4)}")
            time.sleep(self.update_interval*1.5)

    def go_signal_callback(self, msg):
        """Callback for the go signal."""
        if msg.data:  # True: Start execution
            self.get_logger().info("Received 'Go Signal': Starting trajectory execution.")
            if not self.execution_in_progress:
                self.center_received=False
                self.heading_received=False
                self.stop_event.clear()

                self.execution_thread = threading.Thread(target=self.execute)
                self.execution_thread.start()
                self.get_logger().info("Received 'Go Signal': Starting trajectory execution.")
            else:
                self.get_logger().info("Received 'Go Signal'but execution still in progress.")

        else:  # False: Stop execution
            self.get_logger().info("Received 'Stop Signal': Stopping trajectory execution.")
            self.execution_in_progress = False
    def normalize_trajectory_features(self, features):
        try:
        # Handle NaN values first
            features = np.nan_to_num(features)
            
            mean = np.mean(features, axis=0)
            std = np.std(features, axis=0, ddof=1)
            
            std = np.where(std < 1e-10, 1.0, std)
            
            normalized_features = (features - mean) / std
            
            if np.any(np.isnan(normalized_features)):
                self.get_logger().warning("NaN values detected in normalized features")
                normalized_features = np.nan_to_num(normalized_features)
                
            return normalized_features
            
        except Exception as e:
            self.get_logger().error(f'Error in normalization: {str(e)}')
            return features
    def execute(self):
        """Main function to compute and send the trajectory."""
        home_lat=-35.363227
        home_lon=149.165237
        home_alt=589
        if  not self.execution_in_progress:
            self.execution_in_progress=True
            self.get_logger().info("Waiting for position and heading data...")
            print(self.heading_received,self.center_received)
            if self.stop_event.is_set():
                    return
            while not (self.center_received and self.heading_received):
                print(self.heading_received,self.center_received)
                if self.stop_event.is_set():
                    return
                time.sleep(0.1)
            
            center = np.array([self.current_x, self.current_y, self.current_z])
            print(f"center {center} heeading {self.current_heading}")
            self.current_heading=0
            body_points = generate_eight_in_body_frame(self.amplitude, self.length, self.points)
            global_points = transform_to_global(body_points, self.current_heading, center, self.angel_x,self.angel_y, self.angle_z )
            for i in range(global_points.shape[1]):
                print(f"Point {i+1}: X={global_points[0, i]:.2f}, Y={global_points[1, i]:.2f}, Z={global_points[2, i]:.2f}")
            if self.stop_event.is_set():
                    return
            visualize_trajectory(global_points, center, self.current_heading)
            print(np.shape(global_points))
            # lats,lons,alts=enu_to_geodetic(global_points,home_lat,home_lon,home_alt)
            # visualize_geodetic_points(lats,lons,alts,home_lat,home_lon)
            # for lat, lon, alt in zip(lats, lons, alts):
            #     print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
            global_points[[0, 1], :] = global_points[[1, 0], :]
            global_points = np.round(global_points, decimals=3)
            global_points = global_points.T  # Transform from (3,250) to (250,3)

            self.get_logger().info(f"global point shape{np.shape(global_points)}")
            feature=self.extract_advanced_trajectory_features(global_points)
            self.get_logger().info(f"shapes of features {np.shape(feature)}")
            feature=self.normalize_trajectory_features(feature)
            self.get_logger().info(f"shapes of features after normalize {np.shape(feature)}")

            self.trajectory_publisher(feature)
            global_points = global_points.T  # Transform from (3,250) to (250,3)

            self.send_trajectory_to_drone(global_points,584.15)
            time.sleep(5)
            self.get_logger().info("Execution is done.")

            self.execution_in_progress = False
    def trajectory_publisher(self,features):
        msg = Float32MultiArray()
        msg.data = features.tolist()  # Convert NumPy array to list for ROS compatibility

        # Publish the message
        self.feature_publisher.publish(msg)
        self.get_logger().info(f'Published features: {msg.data}')

import signal  # Added for KeyboardInterrupt handling


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    def signal_handler(sig, frame):
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
        node.stop_event.set()
        if hasattr(node, 'execution_thread') and node.execution_thread.is_alive():
            node.execution_thread.join()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



 # def send_trajectory_to_drone(self, global_points):
    #     """Send the trajectory points to the drone."""
    #     self.get_logger().info("Sending trajectory points to the drone...")
    #     ignore_velocity = (
    #         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    #         | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    #         | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    #         | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    #              | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

    #     )

    #     ignore_accel = (
    #         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    #         | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    #         | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    #     )
    #     global self.current_heading
    #     print(self.current_heading)
    #     for i in range(global_points.shape[1] - 1):
    #         # Calculate relative offsets
    #         dx = global_points[0, i + 1] - global_points[0, i]
    #         dy = global_points[1, i + 1] - global_points[1, i]
    #         dz = global_points[2, i + 1] - global_points[2, i]

    #         # Send the offset command
    #         self.drone.mav.set_position_target_local_ned_send(
    #             0,
    #             self.drone.target_system,
    #             self.drone.target_component,
    #             mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    #             (ignore_velocity | ignore_accel),
    #             dx, dy, dz,
    #             0, 0, 0,
    #             0,0,0,
    #             self.current_heading, 0
    #         )
    #         self.get_logger().info(f"Sent offset: dx={dx}, dy={dy}, dz={dz}")
    #         time.sleep(UPDATE_INTERVAL)
