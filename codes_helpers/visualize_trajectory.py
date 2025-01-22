import pandas as pd

# Load the CSV file to inspect its contents
file_path = '/home/ibu/bitirme_ws/noise_20_140_100/150point_noise_20_100_shape_3_3_loc_-1_0_x90/data/iris1_XYZ.csv'
data = pd.read_csv(file_path)

# Display the first few rows of the data to understand its structure
data.head()


import numpy as np
import matplotlib.pyplot as plt

# Extract the X, Y, Z position data
global_points = data[['Position X', 'Position Y', 'Position Z']].to_numpy().T
global_points = np.hstack((global_points, global_points[:, [0]]))
# Define a dummy center and heading for demonstration
center = global_points[:, 0]  # Using the first point as the center
heading = 0  # Assuming a 45-degree heading

# Define the visualization function
def visualize_trajectory(global_points, center, heading):
    """Visualize the eight trajectory and the drone's heading in 3D."""
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the trajectory in the global frame
    ax.plot(global_points[0, :], global_points[1, :], global_points[2, :],
            label="Eight Trajectory", color="b")

    # Plot the drone's position
    # ax.scatter(center[0], center[1], center[2], color="r", label="Drone Position", s=50)

    # Add drone heading as an arrow
    heading_rad = np.radians(heading)
    arrow_length = 10.0  # Length of the arrow
    dx = arrow_length * np.cos(heading_rad)
    dy = arrow_length * np.sin(heading_rad)
    dz = 0  # No vertical change for heading arrow

    # ax.quiver(center[0], center[1], center[2], dx, dy, dz, color="g",
            #   label="Drone Heading", arrow_length_ratio=0.2)

    # Set labels and title
    ax.set_title("Eight Trajectory Visualization", fontsize=14)
    ax.set_xlabel("X (North) [m]")
    ax.set_ylabel("Y (East) [m]")
    ax.set_zlabel("Z (Altitude) [m]")
    ax.legend()

    plt.show()

# Visualize the trajectory
visualize_trajectory(global_points, center, heading)
