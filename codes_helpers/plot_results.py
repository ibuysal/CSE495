import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ast

# Load the data
df = pd.read_csv("/home/ibu/bitirme_ws/gps_deneme_package/all_data_loc_image_input.csv")
columns_to_convert = ["Real Pose", "Predicted Pose", "Start Pose Iris", "Start Pose Iris1"]  # Replace with your column names
for column in columns_to_convert:
    df[column] = df[column].apply(ast.literal_eval)

# Add a processed noise level column for visualization
df["Processed Noise Levels"] = df["Noise Levels"].apply(lambda x: ast.literal_eval(x))
# Extract data for plotting
real_x = [Position[0] for Position in df["Real Pose"]]  # North
real_y = [Position[1] for Position in df["Real Pose"]]  # East
real_z = [Position[2] for Position in df["Real Pose"]]  # Altitude
predicted_x = [Position[0] for Position in df["Predicted Pose"]]
predicted_y = [Position[1] for Position in df["Predicted Pose"]]
predicted_z = [Position[2] for Position in df["Predicted Pose"]]

# Plot Real vs Predicted for each axis
axes = ['North', 'East', 'Altitude']
real_values = [real_x, real_y, real_z]
predicted_values = [predicted_x, predicted_y, predicted_z]

for i, axis in enumerate(axes):
    plt.figure(figsize=(10, 6))
    plt.scatter(real_values[i], predicted_values[i], color='blue', label=f"Predicted vs Real ({axis})")

    # Add reference line (y = x)
    min_val = min(min(real_values[i]), min(predicted_values[i]))
    max_val = max(max(real_values[i]), max(predicted_values[i]))
    plt.plot([min_val, max_val], [min_val, max_val], 'r--', label="Reference Line (y=x)")

    # Annotate points with indices as per real points array
    for idx, (real, pred) in enumerate(zip(real_values[i], predicted_values[i])):
        plt.text(real, pred, f"{idx + 1}", fontsize=9, color='black', ha='right')

    # Calculate and display errors for the graph
    errors = [np.abs(real - pred) for real, pred in zip(real_values[i], predicted_values[i])]
    avg_error = np.mean(errors)
    plt.text(min_val, max_val * 0.9, f"Avg Error: {avg_error:.4f}", fontsize=12, color='red')
    print(f"Axis: {axis}, Average Error: {avg_error:.4f}")

    plt.xlabel(f"Real {axis} (m)")
    plt.ylabel(f"Predicted {axis} (m)")
    plt.title(f"Real vs Predicted {axis}")
    plt.legend()
    plt.grid()
    plt.show()

# Group errors by noise levels (N1)
df["Error"] = np.sqrt((df["Real Pose"].apply(lambda x: x[0]) - df["Predicted Pose"].apply(lambda x: x[0]))**2 +
                       (df["Real Pose"].apply(lambda x: x[1]) - df["Predicted Pose"].apply(lambda x: x[1]))**2 +
                       (df["Real Pose"].apply(lambda x: x[2]) - df["Predicted Pose"].apply(lambda x: x[2]))**2)

df["N1 Noise"] = df["Processed Noise Levels"].apply(lambda x: x['iris_1_noise'])

# Plot each point's error grouped by N1 Noise Level
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

    # Print average error for the noise level
    avg_error = np.mean(y_positions)
    print(f"Noise Level: {noise}, Average Error: {avg_error:.4f}")
    plt.text(noise, max(y_positions) * 0.9, f"Avg Error: {avg_error:.4f}", fontsize=10, color='red', ha='center')

plt.xlabel("N1 Noise Level")
plt.ylabel("Error")
plt.title("Error for Each Point by N1 Noise Level")
plt.legend()
plt.grid()
plt.show()
