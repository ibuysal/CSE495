import numpy as np
import matplotlib.pyplot as plt

# Parameters for the eight trajectory
AMPLITUDE = 5.0  # Half-width of the rectangle
LENGTH = 10.0  # Half-length of the rectangle
POINTS = 100  # Number of points in the trajectory
ANGLE_Y = 45  # Inclination angle of the rectangle in degrees (around Y-axis)
ANGLE_Z = 0  # Orientation angle of the rectangle in degrees (around Z-axis)
HEADING = 95  # Drone's heading in degrees (relative to North)
drone_position = np.array([0, 0, 10])  # Drone's position in the global frame (X, Y, Z)

# Rotation matrices
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
def transform_to_global(body_points, heading, position, angle_y, angle_z):
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

    # Combine rotations
    total_rotation = heading_rotation @ inclination_rotation @ orientation_rotation

    # Transform points
    global_points = total_rotation @ body_points

    # Translate points to the drone's position
    global_points[0, :] += position[0]
    global_points[1, :] += position[1]
    global_points[2, :] += position[2]

    return global_points

# Plot the rectangle
def plot_rectangle(ax, position, heading, amplitude, length, angle_y, angle_z):
    """Plot the inclined rectangle in 3D."""
    # Define rectangle corners in the body frame
    corners_body = np.array([
        [-length, -amplitude, 0],
        [-length, amplitude, 0],
        [length, amplitude, 0],
        [length, -amplitude, 0],
        [-length, -amplitude, 0]  # Close the rectangle
    ]).T  # Shape (3, 5)
    
    # Rotate the rectangle in the body frame
    inclination_rotation = rotation_matrix_y(angle_y)
    orientation_rotation = rotation_matrix_z(angle_z)
    corners_body_rotated = orientation_rotation @ inclination_rotation @ corners_body

    # Transform to the global frame
    heading_rad = np.radians(heading)
    heading_rotation = np.array([
        [np.cos(heading_rad), -np.sin(heading_rad), 0],
        [np.sin(heading_rad), np.cos(heading_rad), 0],
        [0, 0, 1]
    ])
    corners_global = heading_rotation @ corners_body_rotated

    # Translate to the drone's position
    corners_global[0, :] += position[0]
    corners_global[1, :] += position[1]
    corners_global[2, :] += position[2]

    # Plot the rectangle
    ax.plot(corners_global[0, :], corners_global[1, :], corners_global[2, :],
            label="Rectangle", color="g", linestyle="--")

# Plot the drone's heading
def plot_drone_heading(ax, position, heading, length=5):
    """Plot an arrow showing the drone's heading."""
    # Heading direction in the global frame
    heading_rad = np.radians(heading)
    direction = np.array([length * np.cos(heading_rad), length * np.sin(heading_rad), 0])
    end_point = position + direction

    # Plot the arrow
    ax.quiver(position[0], position[1], position[2],
              direction[0], direction[1], direction[2],
              color="r", label="Drone Heading", arrow_length_ratio=0.2)

# Generate eight trajectory in the body frame
body_points = generate_eight_in_body_frame(AMPLITUDE, LENGTH, POINTS)

# Transform trajectory to global frame
global_points = transform_to_global(body_points, HEADING, drone_position, ANGLE_Y, ANGLE_Z)

# Visualization
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection="3d")

# Plot the trajectory in the global frame
ax.plot(global_points[0, :], global_points[1, :], global_points[2, :], label="Eight Trajectory", color="b")

# Plot the drone's position and heading
plot_drone_heading(ax, drone_position, HEADING)

# Plot the rectangle
plot_rectangle(ax, drone_position, HEADING, AMPLITUDE, LENGTH, ANGLE_Y, ANGLE_Z)

# Labels and title
ax.set_title("Eight Trajectory with Rectangle and Drone Heading", fontsize=14)
ax.set_xlabel("X (North) [m]")
ax.set_ylabel("Y (East) [m]")
ax.set_zlabel("Z (Altitude) [m]")
ax.legend()

plt.show()
