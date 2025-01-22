#!/usr/bin/env python3
import os
# Disable CUDA and suppress TF warnings
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import cv2
import os
from gps_deneme_package.helper.common import *
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float32MultiArray
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import time 
import pandas as pd
import sys
def cumulative_arc_lengths(points):
    """
    points: a list of (x, y)
    return: 
      lengths: array of cumulative distances, length = number_of_points
               lengths[0] = 0
               lengths[-1] = total arc length
    """
    lengths = [0.0]
    for i in range(1, len(points)):
        (x1, y1) = points[i-1]
        (x2, y2) = points[i]
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        lengths.append(lengths[-1] + dist)
    return np.array(lengths)

def resample_points(points, num_output=240):
    """
    Resample the trajectory points at num_output+1 param steps from 0 to 1 inclusive.
    Return num_output+1 new points.
    """
    cums = cumulative_arc_lengths(points)
    L = cums[-1]  # total length

    new_u = np.linspace(0.0, 1.0, num_output+1)  # e.g. 241 points
    new_points = []

    for u in new_u:
        target_length = u * L

        idx = np.searchsorted(cums, target_length)
        if idx <= 0:
            new_points.append(points[0])
        elif idx >= len(points):
            new_points.append(points[-1])
        else:
            t = (target_length - cums[idx-1]) / (cums[idx] - cums[idx-1])
            x1, y1 = points[idx-1]
            x2, y2 = points[idx]
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            new_points.append((x, y))

    return new_points

def remove_outliers(points, method="iqr", threshold=1.5):
    """
    Remove outliers from 2D trajectory points.
    
    Args:
        points (np.ndarray or pd.DataFrame): Nx2 array of [x, y] points.
        method (str): "iqr" or "zscore". Method for outlier detection.
        threshold (float): Threshold for outlier detection.
            - For "iqr", it's the IQR multiplier (default 1.5).
            - For "zscore", it's the Z-score threshold (default 3.0).
    
    Returns:
        np.ndarray: Cleaned [x, y] points without outliers.
    """
    if isinstance(points, np.ndarray):
        df = pd.DataFrame(points, columns=["x", "y"])
    elif isinstance(points, pd.DataFrame):
        df = points.copy()
    else:
        raise ValueError("Input points should be a NumPy array or pandas DataFrame.")
    
    if method == "iqr":
        Q1 = df.quantile(0.25)
        Q3 = df.quantile(0.75)
        IQR = Q3 - Q1
        
        lower_bound = Q1 - threshold * IQR
        upper_bound = Q3 + threshold * IQR
        
        mask = ((df >= lower_bound) & (df <= upper_bound)).all(axis=1)
    elif method == "zscore":
        z_scores = (df - df.mean()) / df.std()
        
        mask = (z_scores.abs() < threshold).all(axis=1)
    else:
        raise ValueError("Invalid method. Use 'iqr' or 'zscore'.")
    
    return df[mask].to_numpy()
import matplotlib.pyplot as plt


def get_drone_center(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # plt.imshow(image)  # Display the image
    # plt.title("Masked Trajectory")  # Add a title
    # plt.axis("off")  # Hide the axes for a cleaner look
    # plt.show()
    lower1 = np.array([0, 19, 0])
    upper1 = np.array([19,255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)

    lower2 = np.array([156,19,0])
    upper2 = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower2, upper2)
    
    # Combine masks
    combined_mask = cv2.bitwise_or(mask1, mask2)
    
    result1 = cv2.bitwise_and(image, image, mask=mask1)
    result2 = cv2.bitwise_and(image, image, mask=mask2)
    combined_result = cv2.bitwise_and(image, image, mask=combined_mask)
    kernel = np.ones((1,1), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    max_area = 0
    largest_box = None
    
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if area > max_area:
            max_area = area
            largest_box = (x, y, w, h)

    if largest_box != None:
        return (int(largest_box[0]+(largest_box[2]/2)), largest_box[1])#     cv2.circle(image, (int(largest_box[0]+(largest_box[2]/2)), largest_box[1]),2, (255, 255, 255), 2)
    else:
        return (0,0)
import matplotlib.pyplot as plt
import threading
from tensorflow.keras.models import load_model
import tensorflow as tf
from tensorflow.keras import layers, models, Input
from tensorflow.keras.applications import MobileNetV2

from tensorflow.keras.applications.mobilenet_v2 import preprocess_input

def distance_loss(y_true, y_pred):
    """
    Custom loss function to compute the Euclidean distance
    between predicted and target positions.

    Args:
        y_true: Tensor of true positions, shape (batch_size, 3).
        y_pred: Tensor of predicted positions, shape (batch_size, 3).

    Returns:
        Mean Euclidean distance over the batch.
    """
    squared_differences = tf.square(y_pred - y_true)

    sum_squared_differences = tf.reduce_sum(squared_differences, axis=-1)

    distances = tf.sqrt(sum_squared_differences)

    return tf.reduce_mean(distances)
def mean_euclidean_distance(y_true, y_pred):
    """
    Compute the mean Euclidean distance directly on unscaled coordinates.
    
    Args:
    y_true: Ground truth coordinates (batch_size, 3).
    y_pred: Predicted coordinates (batch_size, 3).
    
    Returns:
    Mean Euclidean distance as a scalar value.
    """
    squared_diff = tf.square(y_pred - y_true)
    
    sum_squared = tf.reduce_sum(squared_diff, axis=-1)
    
    distances = tf.sqrt(sum_squared)
    
    return tf.reduce_mean(distances)
class KerasInferenceNode(Node):
    def __init__(self):
        super().__init__('keras_inference_node')
        
        tf.config.set_visible_devices([], 'GPU')
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
        
        tf.keras.utils.disable_interactive_logging()
        import keras.config
        keras.config.enable_unsafe_deserialization()
        
        self.bridge = CvBridge()
        

        self.just_image=False

        if not self.just_image:

            try:
                self.model = load_model('/home/ibu/bitirme_ws/model1.keras', custom_objects={'distance_loss': distance_loss,'mean_euclidean_distance':mean_euclidean_distance})
                self.model.summary()

                self.get_logger().info('Model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load model: {str(e)}')
                raise  
        else:
            try:
                print("just image")
                self.model = load_model('/home/ibu/bitirme_ws/model2.keras', custom_objects={'distance_loss': distance_loss,'mean_euclidean_distance':mean_euclidean_distance})
                self.model.summary()

                self.get_logger().info('Model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load model: {str(e)}')
                raise  
        
        # Initialize subscribers
        image_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        pose_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        pose_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/image",
            qos_profile=image_qos
        )

        self.pose_sub = message_filters.Subscriber(
            self,
            PoseStamped,
            '/iris_local_ned',
            qos_profile=pose_qos
        )
        self.iris1_pose_sub = message_filters.Subscriber(
            self,
            PoseStamped,
            '/ap/pose/filtered', 
            qos_profile=pose_qos
        )
        self.traj_sub = self.create_subscription(
            
            PointStamped,
            "iris_1_x_y_z",
            self.traj_callback,
            10
        )

        
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.pose_sub,self.iris1_pose_sub],
            queue_size=10,
            slop=0.1  
        )
        self.sync.registerCallback(self.synchronized_callback)

        self.go_signal_subscriber = self.create_subscription(
            Bool,
            IRIS_1_GO_SIGNAL,
            self.go_signal_callback,
            10
        )
        self.execution_in_progress=False
        self.prediction_publisher = self.create_publisher(Float32MultiArray, '/predicted_iris1_pose', 10)

        
        self.latest_image = None
        self.observer_pose = None
        self.features=None
        self.image_iris_pose=[]
        self.trajectory=[]
        self.is_sleep_activated=False
        self.xyz_counter=0
        self.get_logger().info('Keras Inference Node initialized')
        self.masked_traj_predict=None
        self.iris_start_end_predict=None
        self.iris1_start_end_predict=None
        
    def go_signal_callback(self, msg):
        """Callback for the go signal."""
        if msg.data:  # True: Start execution
            self.get_logger().info("Received 'Go Signal': Starting trajectory execution.")
            self.latest_image = None
            self.observer_pose = None
            self.features=None
            self.image_iris_pose=[]
            self.trajectory=[]
            # Add subscription list check after creating subscribers
            # Create timer for periodic inference
            # self.create_timer(0.2, self.inference_callback)  # 10Hz
            self.xyz_counter=0
            self.masked_traj_predict=None
            self.iris_start_end_predict=None
            self.iris1_start_end_predict=None
            if not self.execution_in_progress:

                self.execution_in_progress=True
            else:
                self.get_logger().info("Received 'Go Signal'but execution still in progress.")

    def traj_callback(self,msg):

        if msg and (self.execution_in_progress):
            self.xyz_counter+=1 
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            x=msg.point.x # Example x value
            y=msg.point.y   # Example y value
            z=msg.point.z
            print(f"i {self.xyz_counter} x{x},y{y},z{z}")
            dict_traj={}
            dict_traj["Position X"] = x
            dict_traj["Position Y"] = y
            dict_traj["Position Z"] = z
            dict_traj["Sec"]=sec
            dict_traj["Nanosec"]=nanosec
            self.trajectory.append(dict_traj)
            if(self.xyz_counter==150):
                self.process_data()


    def process_data(self):
        print( "process data")
        if not self.is_sleep_activated:
            
            print("Starting process_data thread")
            self.is_sleep_activated = True

            # Start a separate thread for processing
            self.processing_thread = threading.Thread(target=self._process_data_thread)
            self.processing_thread.start()
    def _process_data_thread(self):
        try:
            print("Thread started")
            time.sleep(1.2 * 12)  
            print("thread sleepd")
            self.execution_in_progress = False
            self.image_iris_pose = sorted(self.image_iris_pose, key=lambda x: (x["image_sec"], x["image_nanosec"]))
            start_time = self.trajectory[0]['Sec'] + self.trajectory[0]['Nanosec'] * 1e-9
            end_time = self.trajectory[-1]['Sec'] + self.trajectory[-1]['Nanosec'] * 1e-9 + 1.2
            drone_centers = []
            iris_coords = []
            iris1_coords = []
            print("thread2")
            print(len(self.image_iris_pose))
            a=0
            for data in self.image_iris_pose:
                print(f"{a}")
                a+=1
                current_time = data['image_sec'] + data['image_nanosec'] * 1e-9
                if start_time <= current_time <= end_time:
                    drone_centers.append(get_drone_center(data["image"]))
                    iris_coords.append(data["iris_coords"])
                    iris1_coords.append(data["iris1_coords"])
            print("thread3")

            drone_pixels_outlier = pd.DataFrame(drone_centers).rename(columns={0: 'x', 1: 'y'})
            drone_pixels_outlier = remove_outliers(drone_pixels_outlier)
            resampled_drone_pixels = resample_points(drone_pixels_outlier, num_output=250)

            masked_trajectory = np.zeros((480, 640, 3), dtype=np.uint8)
            for pixel in resampled_drone_pixels:
                x = int(pixel[0])
                y = int(pixel[1])
                cv2.circle(masked_trajectory, (x, y), 1, (255, 0, 0), 2)

            plt.imshow(masked_trajectory) 
            plt.title("Masked Trajectory") 
            plt.axis("off")  
            plt.show()

            self.iris_start_end_predict = (iris_coords[0], iris_coords[-1])
            self.iris1_start_end_predict = (iris1_coords[0], iris1_coords[-1])
            self.masked_traj_predict = masked_trajectory
            print("thread4")

            self.inference()

        except Exception as e:
            self.get_logger().error(f"Error in thread: {str(e)}")

        finally:
            self.is_sleep_activated = False  
            print("Thread finished")
    def synchronized_callback(self, image_msg, pose_msg, iris1_pose_msg):
        if self.execution_in_progress:
            try:
                
                dict={}

                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                # cv_image = cv2.resize(cv_image, (128, 128))
                # cv_image = cv_image.astype(np.float32) / 255.0
                # self.latest_image = cv_image

                iris_coords = np.array([
                    round(pose_msg.pose.position.x,3),
                
                    round(pose_msg.pose.position.y,3),
                    -1*round(pose_msg.pose.position.z,3)
                ])
                iris1_coords = np.array([
                    5+round(iris1_pose_msg.pose.position.y,3),
                    round(iris1_pose_msg.pose.position.x,3),
                    round(iris1_pose_msg.pose.position.z,3)
                ])
                
                print(iris_coords)
                print(iris1_coords)
                print(pose_msg.pose)
                print(iris1_pose_msg.pose)
                self.get_logger().info('Synchronized image and pose processed')
                image_timestamp = image_msg.header.stamp
                pose_timestamp = pose_msg.header.stamp
                iris1_pose_timestamp = iris1_pose_msg.header.stamp
                
                dict["image"]=cv_image
                dict["image_sec"]=image_timestamp.sec
                dict["image_nanosec"]=image_timestamp.nanosec

                dict["iris_coords"]=iris_coords
                dict["iris_sec"]=pose_timestamp.sec
                dict["iris_nanosec"]=pose_timestamp.nanosec

                dict["iris1_coords"]=iris1_coords
                dict["iris1_sec"]=iris1_pose_timestamp.sec
                dict["iris1_nanosec"]=iris1_pose_timestamp.nanosec

                self.image_iris_pose.append(dict)
                image_time_sec = image_timestamp.sec + image_timestamp.nanosec * 1e-9
                pose_time_sec = pose_timestamp.sec + pose_timestamp.nanosec * 1e-9
                iris1_time_sec = iris1_pose_timestamp.sec + iris1_pose_timestamp.nanosec * 1e-9
                current_time_seconds = time.time()

                current_time_seconds_int = int(current_time_seconds)
                current_time_nanoseconds = int((current_time_seconds - current_time_seconds_int) * 1e9)

                # Print the converted times
                # print(
                #     f"Image Time: {image_time_sec:.9f} sec, "
                #     f"Pose Time: {pose_time_sec:.9f} sec, "
                #     f"Pose1 Time: {iris1_time_sec:.9f} sec"
                # )
                # print(f"Seconds: {current_time_seconds_int}, Nanoseconds: {current_time_nanoseconds}")

                # print(f"image sec: {image_timestamp.sec} image nanosec: {image_timestamp.nanosec} pose sec: {pose_timestamp.sec} nanosec:{pose_timestamp.nanosec}  pose1 sec: {iris1_pose_timestamp.sec} nanosec:1{iris1_pose_timestamp.nanosec}")
            except Exception as e:
                self.get_logger().error(f'Error in synchronized callback: {str(e)}')
    def distance_loss(self, y_true, y_pred):
        x_diff = tf.square(y_true[:, 0] - y_pred[:, 0])
        y_diff = tf.square(y_true[:, 1] - y_pred[:, 1])
        z_diff = tf.square(y_true[:, 2] - y_pred[:, 2])
        weighted_distance = tf.sqrt(x_diff + y_diff + z_diff)
        return tf.reduce_mean(weighted_distance)

  

    def inference(self):
        if self.masked_traj_predict is None or self.iris1_start_end_predict is None or self.iris_start_end_predict is None:
            return

        try:
            # Prepare inputs for the model
            print("inference")
            resized_image = tf.image.resize(self.masked_traj_predict, (240,320))
            img_masked_array = tf.keras.preprocessing.image.img_to_array(resized_image)
            img_masked_array = img_masked_array / 255.0
            image=preprocess_input(img_masked_array)

            relative_start=self.iris_start_end_predict[0]-self.iris1_start_end_predict[0]
            relative_end=self.iris_start_end_predict[1]-self.iris1_start_end_predict[1]
            print(relative_start,relative_end)
            print(np.shape(image),np.shape(relative_start),np.shape(relative_end))
            image = np.expand_dims(image, axis=0)
            relative_start = np.expand_dims(relative_start, axis=0)
            if not self.just_image:
                prediction = self.model.predict(
                    [image, relative_start],
                    verbose=0
                )
            else:
                prediction = self.model.predict(
                    [image],
                    verbose=0
                )
            x = float(prediction[0][0])
            y = float(prediction[0][1])
            z= float(prediction[0][2])

            print(x,y,z)
            print(self.iris_start_end_predict[1],self.iris1_start_end_predict[1])
            print(type(self.iris_start_end_predict[1]),type(self.iris1_start_end_predict[1]))
            x_y_z=np.array([x,y,z])
            self.publish_prediction(x_y_z,self.iris_start_end_predict[1],self.iris1_start_end_predict[1],self.iris_start_end_predict[0], self.iris1_start_end_predict[0])
            self.latest_image = None
            self.observer_pose = None
            self.features=None
            self.image_iris_pose=[]
            self.trajectory=[]
            # Add subscription list check after creating subscribers
            # Create timer for periodic inference
            # self.create_timer(0.2, self.inference_callback)  # 10Hz
            self.xyz_counter=0
            self.masked_traj_predict=None
            self.iris_start_end_predict=None
            self.iris1_start_end_predict=None
            
        except Exception as e:
            self.get_logger().error(f'Error during inference: {str(e)}')
    def publish_prediction(self, predictions, iris_end, iris1_end,iris_start, iris1_start):
        """
        Publishes the prediction data as a Float32MultiArray.

        Args:
            predictions (list): Start coordinates [x, y, z] for the first drone.
            iris_end (list): End coordinates [x, y, z] for the first drone.
            iris1_end (list): End coordinates [x, y, z] for the second drone.
        """
        if not all(isinstance(arr, np.ndarray) for arr in [predictions, iris_end, iris1_end,iris_start, iris1_start]):
            raise ValueError("All inputs must be NumPy arrays")

        data_to_publish = predictions.tolist() + iris_end.tolist() + iris1_end.tolist() +iris_start.tolist() +iris1_start.tolist()

        msg = Float32MultiArray()

        
        data_to_publish = np.concatenate((predictions, iris_end, iris1_end,iris_start,iris1_start)).flatten().tolist()
        print("data to publish ")
        print(data_to_publish)
        print(predictions , iris_end , iris1_end)
        msg.data = data_to_publish

        # Publish the message
        self.prediction_publisher.publish(msg)

        self.get_logger().info(f"Published prediction data: {data_to_publish}")

def main(args=None):
    rclpy.init(args=args)
    node = KerasInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()