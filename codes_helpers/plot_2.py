import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast
df = pd.read_csv("/home/ibu/bitirme_ws/gps_deneme_package/all_data_loc_image_input.csv")
columns_to_convert = ["Real Pose", "Predicted Pose", "Start Pose Iris", "Start Pose Iris1"]  # Replace with your column names
for column in columns_to_convert:
    df[column] = df[column].apply(ast.literal_eval)

# Add a processed noise level column for visualization
df["Processed Noise Levels"] = df["Noise Levels"].apply(lambda x: ast.literal_eval(x))
# Extract data for plotting
start_x = [Position[0] for Position in df["Start Pose Iris1"]]  # North
start_y = [Position[1] for Position in df["Start Pose Iris1"]]  # East
start_z = [Position[2] for Position in df["Start Pose Iris1"]]  # Altitude
real_x = [Position[0] for Position in df["Real Pose"]]  # North
real_y = [Position[1] for Position in df["Real Pose"]]  # East
real_z = [Position[2] for Position in df["Real Pose"]]  # Altitude
predicted_x = [Position[0] for Position in df["Predicted Pose"]]
predicted_y = [Position[1] for Position in df["Predicted Pose"]]
predicted_z = [Position[2] for Position in df["Predicted Pose"]]



df["Error"] = np.sqrt((df["Real Pose"].apply(lambda x: x[0]) - df["Predicted Pose"].apply(lambda x: x[0]))**2 +
                       (df["Real Pose"].apply(lambda x: x[1]) - df["Predicted Pose"].apply(lambda x: x[1]))**2 +
                       (df["Real Pose"].apply(lambda x: x[2]) - df["Predicted Pose"].apply(lambda x: x[2]))**2)

df["N1 Noise"] = df["Processed Noise Levels"].apply(lambda x: x['iris_1_noise'])

grouped = df.groupby("N1 Noise")["Error"].mean()

axes = ['North', 'East', 'Altitude']
real_values = [real_x, real_y, real_z]
predicted_values = [predicted_x, predicted_y, predicted_z]
start_values=[start_x,start_y,start_z]
for i, axis in enumerate(axes):
    plt.figure(figsize=(10, 6))
    plt.scatter(real_values[i], predicted_values[i], color='blue', label=f"Predicted vs Real ({axis})")

    # Add reference line (y = x)
    min_val = min(min(real_values[i]), min(predicted_values[i]))
    max_val = max(max(real_values[i]), max(predicted_values[i]))
    plt.plot([min_val, max_val], [min_val, max_val], 'r--', label="Reference Line (y=x)")

    plt.xlabel(f"Real {axis} (m)")
    plt.ylabel(f"Predicted {axis} (m)")
    plt.title(f"Real vs Predicted {axis}")
    plt.legend()
    plt.grid()
    plt.show()


for i, axis in enumerate(axes):
    plt.figure(figsize=(10, 6))
    plt.scatter(start_values[i], real_values[i], color='blue', label=f"Goal vs Real ({axis})")

    # Add reference line (y = x)
    min_val = min(min(start_values[i]), min(real_values[i]))
    max_val = max(max(start_values[i]), max(real_values[i]))
    plt.plot([min_val, max_val], [min_val, max_val], 'r--', label="Reference Line (y=x)")

    
    plt.xlabel(f"Goal {axis} (m)")
    plt.ylabel(f"Real {axis} (m)")
    plt.title(f"Goal vs Real {axis}")
    plt.legend()
    plt.grid()
    plt.show()



# Plot error vs N1 noise level
unique_noise_levels = df["N1 Noise"].unique()
unique_noise_levels.sort()



plt.figure(figsize=(10, 8))
for noise in unique_noise_levels:
    subset = df[df["N1 Noise"] == noise]
    y_positions = subset["Error"].values
    x_positions = np.full_like(y_positions, noise)
    
    # Scatter each point
    plt.scatter(x_positions, y_positions, label=f"Noise Level {noise}")
    
    # Draw vertical lines and annotate points as per real points array
    for idx, (x, y) in enumerate(zip(x_positions, y_positions)):
        plt.plot([x, x], [0, y], 'k--', alpha=0.5)
        plt.text(x, y, f"{subset.index[idx] + 1}", fontsize=9, color='black', ha='right')

plt.xlabel("Target UAV Noise Level")
plt.ylabel("Error")
plt.title("Error for Each Point by Target UAV Noise Level")
plt.legend()
plt.grid()
plt.show()



def plot_positions(start_x, start_y, start_z, real_x, real_y, real_z, predicted_x, predicted_y, predicted_z, indices, title, xlabel, ylabel,distance=0.1, zlabel=None):
    plt.figure(figsize=(12, 8))
    total_error=0
    for i in indices:
        # Draw dashed lines between Goal Position and Real Position
        plt.plot([start_y[i], real_y[i]], [start_x[i], real_x[i]], 'k--', alpha=0.5, label=None)
        # Draw dashed lines between Real Position and Predicted Position
        plt.plot([real_y[i], predicted_y[i]], [real_x[i], predicted_x[i]], 'r--', alpha=0.5, label=None)

        # Plot Goal Position, Real Position, and Predicted Position
        plt.scatter(start_y[i], start_x[i], color='blue', label="Goal Position" if i == indices[0] else None)
        plt.scatter(real_y[i], real_x[i], color='green', label="Real Position" if i == indices[0] else None)
        plt.scatter(predicted_y[i], predicted_x[i], color='red', label="Predicted Position" if i == indices[0] else None)

        # Annotate the points
        plt.text(real_y[i] + distance, real_x[i] + distance, f"{i + 1}", fontsize=10, color='green', ha='left')
        plt.text(predicted_y[i] + distance, predicted_x[i] + distance, f"{i + 1}", fontsize=10, color='red', ha='left')
        plt.text(start_y[i] + distance, start_x[i] + distance, f"{i + 1}", fontsize=10, color='blue', ha='left')
        error = np.sqrt((real_x[i] - predicted_x[i])**2 + (real_y[i] - predicted_y[i])**2 + (real_z[i] - predicted_z[i])**2)
        total_error += error
        plt.text(predicted_y[i], predicted_x[i] - 0.2, f"Error: {error:.2f}", fontsize=9, color='purple')

    # Calculate and display average error
    avg_error = total_error / len(indices)
    plt.text(0.1, 0.9, f"Avg Error: {avg_error:.2f}", fontsize=12, color='red', transform=plt.gca().transAxes)
    print(f"{title} - Avg Error: {avg_error:.2f}")

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if zlabel:
        plt.ylabel(zlabel)
    plt.title(title)
    plt.legend(loc='best')
    plt.grid()
    plt.show()


# Define data subsets
half_length = len(start_x) // 2
first_half_indices = range(half_length)
second_half_indices = range(half_length, len(start_x))

# First plot: North vs East (First Half)
plot_positions(
    start_x, start_y, start_z, real_x, real_y, real_z, predicted_x, predicted_y, predicted_z,
    first_half_indices,
    title="Prediction vs Real Position with Goal Position (North vs East)",
    xlabel="East (m)", ylabel="North (m)"
)

# Second plot: North vs East (Second Half)
plot_positions(
    start_x, start_y, start_z, real_x, real_y, real_z, predicted_x, predicted_y, predicted_z,
    second_half_indices,
    title="Prediction vs Real Position with Goal Position (North vs East)",
    xlabel="East (m)", ylabel="North (m)"
)

# Third plot: Altitude vs East (First Half)
plot_positions(
    start_z, start_y, start_x, real_z, real_y, real_x, predicted_z, predicted_y, predicted_x,
    first_half_indices,
    title="Prediction vs Real Position with Goal Position (Altitude vs East)",
    xlabel="East (m)", ylabel="Altitude (m)",distance=0.05
)

# Fourth plot: Altitude vs East (Second Half)
plot_positions(
    start_z, start_y, start_x, real_z, real_y, real_x, predicted_z, predicted_y, predicted_x,
    second_half_indices,
    title="Prediction vs Real Position with Goal Position (Altitude vs East)",
    xlabel="East (m)", ylabel="Altitude (m)"
)
