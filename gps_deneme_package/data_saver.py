from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import NavSatFix, Image
from geographic_msgs.msg import GeoPoseStamped
from cv_bridge import CvBridge
import pandas as pd
import cv2
import os
import threading
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy
class SaverNode(Node):
    def __init__(self):
        super().__init__('data_saver_node')
        
        self.save_dir = "recorded_data"
        self.recording = False
        self.bridge = CvBridge()
        
        self.iris_data = {'pose': [], 'navsat': [], 'geopose': []}
        self.iris1_data = {'pose': [], 'navsat': [], 'geopose': [], 'point': []}
        
        self.create_subscription(String, '/saver/command', self.command_callback, 10)
        self.create_subscription(String, '/saver/directory', self.directory_callback, 10)
        
        self.setup_subscribers()
        
        self.image_thread = None
        self.image_queue = []
        self.image_lock = threading.Lock()

    def setup_subscribers(self):
        # Iris subscribers
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(PoseStamped, '/iris_local_ned', 
                               self.iris_pose_callback, self.qos_profile)
        self.create_subscription(NavSatFix, '/navsat', 
                               self.iris_navsat_callback, self.qos_profile)
        self.create_subscription(GeoPoseStamped, '/iris_global_position_int', 
                               self.iris_geopose_callback, self.qos_profile)
        
        # Iris1 subscribers
        self.create_subscription(PoseStamped, '/ap/pose/filtered', 
                               self.iris1_pose_callback, self.qos_profile)
        self.create_subscription(NavSatFix, '/navsat_1', 
                               self.iris1_navsat_callback, self.qos_profile)
        self.create_subscription(GeoPoseStamped, '/ap/geopose/filtered', 
                               self.iris1_geopose_callback, self.qos_profile)
        self.create_subscription(PointStamped, '/iris_1_x_y_z', 
                               self.iris1_point_callback, 10)
        
        # Image subscriber
        self.create_subscription(Image, '/camera/image', 
                               self.image_callback, self.qos_profile)

    def command_callback(self, msg):
        print(msg)
        if msg.data == "start":
            self.start_recording()
        elif msg.data == "stop":
            self.stop_recording()

    def directory_callback(self, msg):
        print(msg)

        self.save_dir = msg.data

    def start_recording(self):
        print(self.save_dir)
        
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, "images"), exist_ok=True)
        print(self.save_dir)
        self.iris_data = {'pose': [], 'navsat': [], 'geopose': []}
        self.iris1_data = {'pose': [], 'navsat': [], 'geopose': [], 'point': []}
        
        self.recording = True
        self.image_thread = threading.Thread(target=self.image_saver_thread)
        self.image_thread.start()
        print(f" recording started")


    def stop_recording(self):
        print("stop recoridng")
        self.recording = False
        if self.image_thread:
            self.image_thread.join()
        self.save_all_data()

    def image_saver_thread(self):
        while self.recording or self.image_queue:
            with self.image_lock:
                if self.image_queue:
                    timestamp, cv_image = self.image_queue.pop(0)
                    try:
                        filename = os.path.join(self.save_dir, "images", f"{timestamp}.jpg")
                        cv2.imwrite(filename, cv_image)
                    except Exception as e:
                        self.get_logger().error(f"Failed to save image: {e}")
    def iris_pose_callback(self, msg):
        if not self.recording:
            return
            
        self.iris_data['pose'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Position_X': msg.pose.position.x,
            'Position_Y': msg.pose.position.y,
            'Position_Z': msg.pose.position.z
        })

    def iris_navsat_callback(self, msg):
        if not self.recording:
            return
            
        self.iris_data['navsat'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Latitude': msg.latitude,
            'Longitude': msg.longitude,
            'Altitude': msg.altitude
        })

    def iris_geopose_callback(self, msg):
        if not self.recording:
            return
            
        self.iris_data['geopose'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Latitude': msg.pose.position.latitude,
            'Longitude': msg.pose.position.longitude,
            'Altitude': msg.pose.position.altitude
        })

    def iris1_point_callback(self, msg):
        if not self.recording:
            return
            
        self.iris1_data['point'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Position X': msg.point.x,
            'Position Y': msg.point.y,
            'Position Z': msg.point.z
        })

    # Callbacks for iris1 (drone 1)
    def iris1_pose_callback(self, msg):
        if not self.recording:
            return
            
        self.iris1_data['pose'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Position_X': msg.pose.position.x,
            'Position_Y': msg.pose.position.y,
            'Position_Z': msg.pose.position.z
        })

    def iris1_navsat_callback(self, msg):
        if not self.recording:
            return
            
        self.iris1_data['navsat'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Latitude': msg.latitude,
            'Longitude': msg.longitude,
            'Altitude': msg.altitude
        })

    def iris1_geopose_callback(self, msg):
        if not self.recording:
            return
            
        self.iris1_data['geopose'].append({
            'Sec': msg.header.stamp.sec,
            'Nanosec': msg.header.stamp.nanosec,
            'Latitude': msg.pose.position.latitude,
            'Longitude': msg.pose.position.longitude,
            'Altitude': msg.pose.position.altitude
        })
    def image_callback(self, msg):
        if not self.recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
            with self.image_lock:
                self.image_queue.append((timestamp, cv_image))
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    # ... [Other callback methods remain same as before] ...

    def save_all_data(self):
        try:
            # Save iris data
            for data_type, data in self.iris_data.items():
                if data:
                    df = pd.DataFrame(data)
                    df.to_csv(os.path.join(self.save_dir, f'iris_{data_type}.csv'), 
                             index=False)
                    print(os.path.join(self.save_dir, f'iris_{data_type}.csv'))
            # Save iris1 data
            for data_type, data in self.iris1_data.items():
                if data:
                    df = pd.DataFrame(data)
                    df.to_csv(os.path.join(self.save_dir, f'iris1_{data_type}.csv'), 
                             index=False)
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")
        print("saved all data ")
def main(args=None):
    rclpy.init(args=args)
    node = SaverNode()
    
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
