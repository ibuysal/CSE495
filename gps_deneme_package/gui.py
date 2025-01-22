import sys
import os 
import pandas as pd
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QFormLayout,
    QGroupBox

)
from PyQt5.QtCore import Qt,QTimer
from std_msgs.msg import Bool, Float32, Float32MultiArray
from std_msgs.msg import String

from geometry_msgs.msg import Point, PoseStamped
from pymavlink import mavutil
from gps_deneme_package.helper.common import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ParameterPublisher(Node):
    def __init__(self):
        super().__init__('parameter_publisher')
        self.parameter_publisher = self.create_publisher(Float32MultiArray, MOVEMENT_PARAMS, 10)
        self.iris_1_noise_publisher = self.create_publisher(Float32, IRIS_1_GPS_NOISE_LEVEL_TOPIC_NAME, 10)
        self.iris_0_noise_publisher = self.create_publisher(Float32, IRIS_GPS_NOISE_LEVEL_TOPIC_NAME, 10)
        self.go_publisher = self.create_publisher(Bool, IRIS_1_GO_SIGNAL, 10)
        self.takeoff_publisher = self.create_publisher(Bool, TAKEOFF_SIGNAL, 10)
        self.xy_publisher = self.create_publisher(Point, IRIS_1_XY, 10)
        self.xy0_publisher = self.create_publisher(Point, IRIS_XY, 10)

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE,     
            depth=10                                  
        )
        #  subscriber for trajectory poses
        self.trajectory_subscription = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.trajectory_callback,
            self.qos_profile
        )
        self.trajectory_points = []
        self.recording = False
        self.predicted_pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/predicted_iris1_pose',
            self.predicted_pose_callback,
            10
        )

        #   poses and errors
        self.latest_real_pose = None  
        self.latest_predicted_pose = None  
        self.error_list = []  #
        self.get_logger().info("Parameter publisher node initialized.")

        # self.saver = DirectSaver(save_dir="recorded_data")
        self.saver_command_pub = self.create_publisher(String, '/saver/command', 10)
        self.saver_directory_pub = self.create_publisher(String, '/saver/directory', 10)
        self.start_poses = [] 
        self.predictions = [] 
        self.real_poses = []  
        self.errors = []  
        self.noise_levels = []
        self.current_iris_0_noise = 0.0
        self.current_iris_1_noise = 0.0
    def predicted_pose_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn("Received predicted pose with insufficient data.")
            return
        #  predicted position
        print(f"data predicted{msg.data}")
        # predict, iris_end, iris1_end

        predicted_relative = np.array(msg.data[:3])
        iris_pose = np.array(msg.data[3:6])
        iris1_pose = np.array(msg.data[6:9])
        iris_start = np.array(msg.data[9:12])
        iris1_start = np.array(msg.data[12:15])

        predicted_pose = iris_pose - predicted_relative
        error_vector = predicted_pose - iris1_pose
        error_distance = np.linalg.norm(error_vector)

        self.start_poses.append({
            'iris_start': iris_start.tolist(),
            'iris1_start': iris1_start.tolist()
        })
        self.predictions.append(predicted_pose.tolist())
        self.real_poses.append(iris1_pose.tolist())
        self.errors.append(error_distance)
        self.noise_levels.append({
            'iris_0_noise': self.get_current_noise_level('iris_0'),
            'iris_1_noise': self.get_current_noise_level('iris_1')
        })

        self.get_logger().info(
            f"Predicted: {predicted_pose}, Real: {iris1_pose}, "
            f"Error: {error_distance:.4f}, Noise: {self.noise_levels[-1]}"
        )


    def get_current_noise_level(self, uav_name):
        """
        Get the current noise level for the specified UAV.
        """
        if uav_name == 'iris_0':
            return self.current_iris_0_noise
        elif uav_name == 'iris_1':
            return self.current_iris_1_noise
        else:
            return 0.0
    def get_latest_real_pose(self):

        return self.real_poses

    def get_latest_predicted_pose(self):
        
        return self.predictions

    def get_mean_error(self):
        if self.errors:
            return np.mean(self.errors)
        return 0.0 
    def get_last_error(self):
        if self.errors:
            return self.errors[-1]
    
   
    def get_start_poses(self):
        return self.start_poses     
    def trajectory_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info(self.recording)
        if self.recording:
            # Extract position from PoseStamped message
            point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.trajectory_points.append(point)

    def start_recording(self):
        self.trajectory_points = []
        self.recording = True
        self.get_logger().info("Started recording trajectory points")

    def stop_recording(self):
        self.recording = False
        self.get_logger().info("Stopped recording trajectory points")

    def get_trajectory_points(self):
        return np.array(self.trajectory_points).T if self.trajectory_points else None

    def visualize_trajectory(self):
        points = self.get_trajectory_points()
        if points is not None and points.shape[1] > 0:
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111, projection="3d")

            # Plot the trajectory
            ax.plot(points[0, :], points[1, :], points[2, :],
                    label="Drone Trajectory", color="b")

            ax.set_title("Drone Trajectory Visualization", fontsize=14)
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Z [m]")
            ax.legend()

            plt.show()
        else:
            print("No trajectory points recorded")

    def publish_parameters(self, parameters):
        msg = Float32MultiArray()
        msg.data = parameters
        self.parameter_publisher.publish(msg)
        self.get_logger().info(f"Published parameters: {parameters}")

    def publish_iris_1_noise_level(self, value):
        self.current_iris_1_noise = value

        msg = Float32()
        msg.data = value
        self.iris_1_noise_publisher.publish(msg)
        self.get_logger().info(f"Published GPS noise level for iris_1: {value}")

    def publish_iris_0_noise_level(self, value):
        self.current_iris_0_noise = value

        msg = Float32()
        msg.data = value
        self.iris_0_noise_publisher.publish(msg)
        self.get_logger().info(f"Published GPS noise level for iris_0: {value}")

    def send_go_signal(self, go_signal):
        msg = Bool()
        msg.data = go_signal
        self.go_publisher.publish(msg)
        self.get_logger().info(f"Sent 'go' signal: {go_signal}")

    def send_takeoff_signal(self, takeoff_signal):
        msg = Bool()
        msg.data = takeoff_signal
        self.takeoff_publisher.publish(msg)
        self.get_logger().info(f"Sent 'takeoff' signal: {takeoff_signal}")

    def publish_xy_coordinates(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.xy_publisher.publish(msg)
        self.get_logger().info(f"Published X, Y coordinates: x={x}, y={y}")
    def publish_iris_0_xy_coordinates(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.xy0_publisher.publish(msg)
        self.get_logger().info(f"Published X, Y coordinates for iris_0: x={x}, y={y}")
    def save_all_data(self, filename="all_data.csv"):
        """
        Save all collected data (start poses, predictions, real poses, errors, and noise levels) to a CSV file.
        """
        try:
            data = []
            for i in range(len(self.predictions)):
                data.append({
                    "Start Pose Iris": self.start_poses[i]['iris_start'],
                    "Start Pose Iris1": self.start_poses[i]['iris1_start'],
                    "Predicted Pose": self.predictions[i],
                    "Real Pose": self.real_poses[i],
                    "Error": self.errors[i],
                    "Noise Levels": self.noise_levels[i]
                })

            df = pd.DataFrame(data)
            df.to_csv(filename, index=False)
            self.get_logger().info(f"All data successfully saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving data to CSV: {e}")


class MainWindow(QWidget):
    def __init__(self, ros2_node):
        super().__init__()
        self.ros2_node = ros2_node
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)  #  every 10ms
        
        self.timer2 = QTimer()
        self.timer2.timeout.connect(self.update_display)
        self.timer2.start(100) 
        self.setWindowTitle("ROS2 Parameter UI")
        self.setGeometry(200, 200, 400, 750) 

        layout = QVBoxLayout()

        # Trajectory Recording Controls
        trajectory_layout = QHBoxLayout()
        
        start_recording_button = QPushButton("Start Target UAV Recording")
        start_recording_button.clicked.connect(self.start_recording)
        trajectory_layout.addWidget(start_recording_button)

        stop_recording_button = QPushButton("Stop Target UAV Recording")
        stop_recording_button.clicked.connect(self.stop_recording)
        trajectory_layout.addWidget(stop_recording_button)

        layout.addLayout(trajectory_layout)

        #  Noise Level Sliders
        iris_1_noise_layout = QVBoxLayout()
        self.iris_1_noise_label = QLabel("GPS Noise Level for Target UAV : 0.0")
        iris_1_noise_layout.addWidget(self.iris_1_noise_label)

        self.iris_1_noise_slider = QSlider(Qt.Horizontal)
        self.iris_1_noise_slider.setMinimum(0)
        self.iris_1_noise_slider.setMaximum(500)
        self.iris_1_noise_slider.setValue(0)
        self.iris_1_noise_slider.valueChanged.connect(self.update_iris_1_noise_level)
        iris_1_noise_layout.addWidget(self.iris_1_noise_slider)

        layout.addLayout(iris_1_noise_layout)

        iris_0_noise_layout = QVBoxLayout()
        self.iris_0_noise_label = QLabel("GPS Noise Level for Observer UAV: 0.0")
        iris_0_noise_layout.addWidget(self.iris_0_noise_label)

        self.iris_0_noise_slider = QSlider(Qt.Horizontal)
        self.iris_0_noise_slider.setMinimum(0)
        self.iris_0_noise_slider.setMaximum(500)
        self.iris_0_noise_slider.setValue(0)
        self.iris_0_noise_slider.valueChanged.connect(self.update_iris_0_noise_level)
        iris_0_noise_layout.addWidget(self.iris_0_noise_slider)

        layout.addLayout(iris_0_noise_layout)

        # Parameter Input Fields
        form_layout = QFormLayout()
        self.amplitude_input = QLineEdit("3.0")
        form_layout.addRow("Amplitude (Half-width):", self.amplitude_input)

        self.length_input = QLineEdit("3.0")
        form_layout.addRow("Length (Half-length):", self.length_input)

        self.points_input = QLineEdit("250")
        form_layout.addRow("Points:", self.points_input)

        self.angle_x_input = QLineEdit("90")
        form_layout.addRow("Angle X (degrees):", self.angle_x_input)

        self.angle_y_input = QLineEdit("0")
        form_layout.addRow("Angle Y (degrees):", self.angle_y_input)

        self.angle_z_input = QLineEdit("0")
        form_layout.addRow("Angle Z (degrees):", self.angle_z_input)

        self.update_interval_input = QLineEdit("0.5")
        form_layout.addRow("Update Interval (seconds):", self.update_interval_input)

        layout.addLayout(form_layout)

        # Buttons
        publish_button = QPushButton("Publish Trajectory Parameters")
        publish_button.clicked.connect(self.publish_parameters)
        layout.addWidget(publish_button)

        takeoff_button = QPushButton("Takeoff")
        takeoff_button.clicked.connect(self.send_takeoff_signal)
        layout.addWidget(takeoff_button)

        go_button = QPushButton("Send 'Go' Signal")
        go_button.clicked.connect(self.send_go_signal)
        layout.addWidget(go_button)

        # X, Y Coordinates

        prediction_group = QGroupBox("Prediction vs Real Pose")
        prediction_layout = QFormLayout()

        # Predicted Pose Labels
        self.pred_x_label = QLabel("N/A")
        self.pred_y_label = QLabel("N/A")
        self.pred_z_label = QLabel("N/A")

        prediction_layout.addRow(QLabel("Predicted X:"), self.pred_x_label)
        prediction_layout.addRow(QLabel("Predicted Y:"), self.pred_y_label)
        prediction_layout.addRow(QLabel("Predicted Z:"), self.pred_z_label)

        # Real Pose Labels
        self.real_x_label = QLabel("N/A")
        self.real_y_label = QLabel("N/A")
        self.real_z_label = QLabel("N/A")

        prediction_layout.addRow(QLabel("Real X:"), self.real_x_label)
        prediction_layout.addRow(QLabel("Real Y:"), self.real_y_label)
        prediction_layout.addRow(QLabel("Real Z:"), self.real_z_label)

        # Mean Error Label
        self.mean_error_label = QLabel("N/A")
        prediction_layout.addRow(QLabel("Mean Error:"), self.mean_error_label)
        self.last_error_label = QLabel("N/A")
        prediction_layout.addRow(QLabel("Last Error:"), self.last_error_label)
        save_csv_button = QPushButton("Save Prediction Data to CSV")
        save_csv_button.clicked.connect(self.save_all_data_to_csv)
        prediction_layout.addWidget(save_csv_button)

        prediction_group.setLayout(prediction_layout)
        layout.addWidget(prediction_group)

        xy_layout = QHBoxLayout()
        self.x_input = QLineEdit("0.0")
        xy_layout.addWidget(QLabel("Target UAV X:"))
        xy_layout.addWidget(self.x_input)

        self.y_input = QLineEdit("0.0")
        xy_layout.addWidget(QLabel("Target UAV Y:"))
        xy_layout.addWidget(self.y_input)

        publish_xy_button = QPushButton("Publish Target UAV X, Y Coordinates")
        publish_xy_button.clicked.connect(self.publish_xy_coordinates)
        xy_layout.addWidget(publish_xy_button)

        layout.addLayout(xy_layout)

        # Add Drone 0 X, Y Coordinate publishing section
        iris_0_xy_layout = QHBoxLayout()
        self.iris_0_x_input = QLineEdit("0.0")
        iris_0_xy_layout.addWidget(QLabel("Observer UAV X:"))
        iris_0_xy_layout.addWidget(self.iris_0_x_input)

        self.iris_0_y_input = QLineEdit("0.0")
        iris_0_xy_layout.addWidget(QLabel("Observer UAV Y:"))
        iris_0_xy_layout.addWidget(self.iris_0_y_input)

        publish_iris_0_xy_button = QPushButton("Publish Observer UAV  X, Y Coordinates")
        publish_iris_0_xy_button.clicked.connect(self.publish_iris_0_xy_coordinates)
        iris_0_xy_layout.addWidget(publish_iris_0_xy_button)
        print("Adding Iris 0 X, Y Coordinate publishing section")

        layout.addLayout(iris_0_xy_layout)

    

        data_set_button = QPushButton("Data Set Prepare Tools")
        data_set_button.clicked.connect(self.toggle_data_set_tools)
        layout.addWidget(data_set_button)
    
        # Create a container for Data Set Prepare Tools layouts
        self.data_set_tools_container = QGroupBox("Data Set Prepare Tools")
        self.data_set_tools_layout = QVBoxLayout()
    
        # CSV Layout
        csv_layout = QHBoxLayout()
        
        self.save_dir_input = QLineEdit("recorded_data")
        csv_layout.addWidget(QLabel("Save Directory:"))
        csv_layout.addWidget(self.save_dir_input)

        start_csv_button = QPushButton("Start CSV Record")
        start_csv_button.clicked.connect(self.start_csv_recording)
        csv_layout.addWidget(start_csv_button)

        stop_csv_button = QPushButton("Stop CSV Record")
        stop_csv_button.clicked.connect(self.stop_csv_recording)
        csv_layout.addWidget(stop_csv_button)

        self.data_set_tools_layout.addLayout(csv_layout)

        align_button = QPushButton("Align Points")
        align_button.clicked.connect(self.run_alignment)
        self.data_set_tools_layout.addWidget(align_button)
        
        # ROS2 Bag Recording Layout
        bag_layout = QHBoxLayout()

        self.bag_file_input = QLineEdit()
        bag_layout.addWidget(self.bag_file_input)

        bag_button = QPushButton("Record Bag")
        bag_button.clicked.connect(self.start_bag_recording)
        bag_layout.addWidget(bag_button)

        stop_bag_button = QPushButton("Stop Bag")
        stop_bag_button.clicked.connect(self.stop_bag_recording)
        stop_bag_button.setEnabled(False)  # Initially disabled
        self.stop_bag_button = stop_bag_button
        bag_layout.addWidget(stop_bag_button)

        self.data_set_tools_layout.addLayout(bag_layout)
    
        # Set the layout for the container and hide it initially
        self.data_set_tools_container.setLayout(self.data_set_tools_layout)
        self.data_set_tools_container.setVisible(False)  # Hidden by default
    
        # Add the container to the main layout
        layout.addWidget(self.data_set_tools_container)

        visualization_layout = QVBoxLayout()
        
        # Directory selection
        dir_select_layout = QHBoxLayout()
        self.viz_dir_input = QLineEdit()
        dir_select_layout.addWidget(QLabel("CSV Directory:"))
        dir_select_layout.addWidget(self.viz_dir_input)
        
        list_csv_button = QPushButton("List CSV Files")
        list_csv_button.clicked.connect(self.list_csv_files)
        dir_select_layout.addWidget(list_csv_button)
        visualization_layout.addLayout(dir_select_layout)
        
        # CSV file selection
        from PyQt5.QtWidgets import QComboBox
        self.csv_selector = QComboBox()
        self.csv_selector.setMinimumWidth(200)
        visualization_layout.addWidget(self.csv_selector)
        
        # Visualization button
        visualize_button = QPushButton("Visualize Selected CSV")
        visualize_button.clicked.connect(self.visualize_selected_csv)
        visualization_layout.addWidget(visualize_button)
        
        layout.addLayout(visualization_layout)

        self.setLayout(layout)
    def update_display(self):
        """
        Periodically called to update the GUI with the latest predicted and real poses and mean error.
        """
        real_pose = self.ros2_node.get_latest_real_pose()
        predicted_pose = self.ros2_node.get_latest_predicted_pose()
        mean_error = self.ros2_node.get_mean_error()
        last_error = self.ros2_node.get_last_error()
        
        if predicted_pose is not None and len(predicted_pose)>0:
            # print(predicted_pose)
            # print(real_pose)
            self.pred_x_label.setText(f"{predicted_pose[len(predicted_pose)-1][0]:.2f}")
            self.pred_y_label.setText(f"{predicted_pose[len(predicted_pose)-1][1]:.2f}")
            self.pred_z_label.setText(f"{predicted_pose[len(predicted_pose)-1][2]:.2f}")
        else:
            self.pred_x_label.setText("N/A")
            self.pred_y_label.setText("N/A")
            self.pred_z_label.setText("N/A")

        if real_pose is not None and len(real_pose)>0:
            self.real_x_label.setText(f"{real_pose[len(real_pose)-1][0]:.2f}")
            self.real_y_label.setText(f"{real_pose[len(real_pose)-1][1]:.2f}")
            self.real_z_label.setText(f"{real_pose[len(real_pose)-1][2]:.2f}")
            # print("error")
            # print(mean_error,last_error,isinstance(mean_error, float),isinstance(last_error, float),type(mean_error),type(last_error))
        else:
            self.real_x_label.setText("N/A")
            self.real_y_label.setText("N/A")
            self.real_z_label.setText("N/A")

        self.mean_error_label.setText(f"{mean_error:.2f}" if isinstance(mean_error, np.float32) else "N/A")
        self.last_error_label.setText(f"{last_error:.2f}" if isinstance(last_error, np.float32) else "N/A")

    def save_all_data_to_csv(self):
        """
        Save all collected data to a CSV file.
        """
        try:
            filename = "all_data.csv"
            self.ros2_node.save_all_data(filename)
            print(f"All data successfully saved to {filename}")
        except Exception as e:
            print(f"Error saving data to CS{e}")
    def toggle_data_set_tools(self):
        """
        Toggle the visibility of the Data Set Prepare Tools container.
        """
        is_visible = self.data_set_tools_container.isVisible()
        self.data_set_tools_container.setVisible(not is_visible)
        print(f"Data Set Prepare Tools visibility set to {not is_visible}")

    def publish_iris_0_xy_coordinates(self):
        try:
            x = float(self.iris_0_x_input.text())
            y = float(self.iris_0_y_input.text())
            self.ros2_node.publish_iris_0_xy_coordinates(x, y)
        except ValueError as e:
            print(f"Invalid input for Iris 0 X, Y coordinates: {e}")

    def list_csv_files(self):
        """
        List all CSV files in the specified directory and populate the combo box
        """
        try:
            directory = self.viz_dir_input.text().strip()
            if not directory:
                print("Please enter a directory path")
                return
                
            if not os.path.exists(directory):
                print(f"Directory {directory} does not exist")
                return
                
            # Clear existing items
            self.csv_selector.clear()
            
            # Find all CSV files in the directory
            csv_files = [f for f in os.listdir(directory) if f.endswith('.csv')]
            
            if not csv_files:
                print("No CSV files found in the directory")
                return
                
            # Add CSV files to combo box
            self.csv_selector.addItems(csv_files)
            print(f"Found {len(csv_files)} CSV files")
            
        except Exception as e:
            print(f"Error listing CSV files: {str(e)}")
            
    def visualize_selected_csv(self):
        """
        Visualize the selected CSV file's trajectory in 3D
        """
        try:
            # Get selected file
            selected_file = self.csv_selector.currentText()
            if not selected_file:
                print("Please select a CSV file")
                return
                
            directory = self.viz_dir_input.text().strip()
            file_path = os.path.join(directory, selected_file)
            
            # Read the CSV file
            df = pd.read_csv(file_path)
            
            # Check if the CSV has the required columns
            required_columns = ['Sec', 'Nanosec', 'Position X', 'Position Y', 'Position Z']
            if not all(col in df.columns for col in required_columns):
                print("Error: CSV file must contain columns: Sec, Nanosec, Position X, Position Y, Position Z")
                return
            
            # Extract position data
            points = np.array([
                df['Position X'].values,
                df['Position Y'].values,
                df['Position Z'].values
            ])
            
            # Create visualization
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111, projection="3d")
            x_range = points[0, :].max() - points[0, :].min()
            y_range = points[1, :].max() - points[1, :].min()
            z_range = points[2, :].max() - points[2, :].min()
            max_range = max(x_range, y_range, z_range)

            # Center the axes
            x_mid = (points[0, :].max() + points[0, :].min()) / 2
            y_mid = (points[1, :].max() + points[1, :].min()) / 2
            z_mid = (points[2, :].max() + points[2, :].min()) / 2

            # Set limits for equal scaling
            ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
            ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
            ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)
            # Plot the trajectory
            ax.plot(points[0, :], points[1, :], points[2, :],
                    label="Drone Trajectory", color="b")

            # Set labels and title
            ax.set_title(f"Trajectory from {selected_file}", fontsize=14)
            ax.set_xlabel("East [m]")
            ax.set_ylabel("North [m]")
            ax.set_zlabel("Altitude [m]")
            ax.legend()

            plt.show()
                
        except FileNotFoundError:
            print(f"Error: Could not find file {file_path}")
        except pd.errors.EmptyDataError:
            print("Error: The CSV file is empty")
        except Exception as e:
            print(f"Error visualizing CSV: {str(e)}")
    def spin_ros(self):
        rclpy.spin_once(self.ros2_node, timeout_sec=0)

    def start_bag_recording(self):
        bag_file = self.bag_file_input.text().strip()
        if not bag_file:
            print("Please enter a bag file name.")
            return
        try:
            self.bag_process = subprocess.Popen(["ros2", "bag", "record", "-o", bag_file, "-a"])
            self.stop_bag_button.setEnabled(True)
            print(f"Started recording bag file: {bag_file}")
        except Exception as e:
            print(f"Failed to start bag recording: {e}")

    def stop_bag_recording(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.bag_process = None
            self.stop_bag_button.setEnabled(False)
            print("Stopped recording bag file.")

    def start_recording(self):
        self.ros2_node.start_recording()

    def stop_recording(self):
        self.ros2_node.stop_recording()
        self.ros2_node.visualize_trajectory()

    # [Previous methods remain the same]
    def update_iris_1_noise_level(self):
        noise_level = self.iris_1_noise_slider.value() / 10.0
        self.iris_1_noise_label.setText(f"GPS Noise Level for iris_1: {noise_level}")
        self.ros2_node.publish_iris_1_noise_level(noise_level)

    def update_iris_0_noise_level(self):
        noise_level = self.iris_0_noise_slider.value() / 10.0
        self.iris_0_noise_label.setText(f"GPS Noise Level for iris_0: {noise_level}")
        self.ros2_node.publish_iris_0_noise_level(noise_level)

    def publish_parameters(self):
        try:
            parameters = [
                float(self.amplitude_input.text()),
                float(self.length_input.text()),
                float(self.points_input.text()),
                float(self.angle_x_input.text()),
                float(self.angle_y_input.text()),
                float(self.angle_z_input.text()),
                float(self.update_interval_input.text()),
            ]
            self.ros2_node.publish_parameters(parameters)
        except ValueError as e:
            print(f"Invalid input: {e}")

    def send_go_signal(self):
        self.ros2_node.send_go_signal(True)
    def start_prediction(self):
        """
        Sends a 'start prediction' signal using the go_publisher.
        """
        try:
            # You can reuse the send_go_signal logic
            self.ros2_node.send_go_signal(True)  # Send the same 'go' signal
            print("Prediction process started.")
        except Exception as e:
            print(f"Error while starting prediction: {e}")
    def send_takeoff_signal(self):
        self.ros2_node.send_takeoff_signal(True)

    def publish_xy_coordinates(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            self.ros2_node.publish_xy_coordinates(x, y)
        except ValueError as e:
            print(f"Invalid input for X, Y coordinates: {e}")
    
    def start_csv_recording(self):
        save_dir = self.save_dir_input.text().strip()
        if not save_dir:
            return
        
        try:
            msg = String()
            msg.data = save_dir
            print(msg)
            self.ros2_node.saver_directory_pub.publish(msg)
            
            msg = String()
            msg.data = "start"
            self.ros2_node.saver_command_pub.publish(msg)
            
        except Exception as e:
            print("ana")

    def stop_csv_recording(self):
        try:
            msg = String()
            msg.data = "stop"
            self.ros2_node.saver_command_pub.publish(msg)
            
            save_dir = self.save_dir_input.text().strip()
        except Exception as e:
            print("ana")
    def run_alignment(self):
        """
        Run the alignment process on the recorded data.
        """
        try:
            base_dir = self.save_dir_input.text().strip()
            if not base_dir:
                print("Please enter a valid base directory")
                return
                
            # Import and run the alignment script
            from image_pose_distance_align import process_directory
            print("Starting alignment process...")
            process_directory(base_dir)
            print("Alignment execution completed successfully")
            
        except ImportError:
            print("Error: Could not import image_pose_distance_align.py. Make sure it's in the same directory.")
        except Exception as e:
            print(f"Error during alignment: {str(e)}")
def main(args=None):
    rclpy.init(args=args)
    ros2_node = ParameterPublisher()

    app = QApplication(sys.argv)
    main_window = MainWindow(ros2_node)
    main_window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()