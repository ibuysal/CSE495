import numpy as np

def fit_pointwise_noise_model(gt_points, noised_points_list):
    """
    Fit a per-point noise model (mean & std) given:
      gt_points:           shape (N, 2) ground-truth
      noised_points_list:  list of arrays [noised0, noised1, ..., noised6],
                           each noised_k has shape (N, 2).
    Returns:
      mean_dx, std_dx, mean_dy, std_dy  (each shape = (N,))
        where mean_dx[i], std_dx[i], mean_dy[i], std_dy[i]
        describe the noise distribution for point i.
    """
    gt_points = np.asarray(gt_points)  # (N, 2)
    n_noisy = len(noised_points_list)  # should be 7 in your case
    
    # For convenience, stack the noisy points into one array of shape (n_noisy, N, 2)
    noisy_stack = np.stack([np.asarray(noised) for noised in noised_points_list], axis=0)
    # noisy_stack.shape = (n_noisy, N, 2)

    N = gt_points.shape[0]
    mean_dx = np.zeros((N,), dtype=np.float32)
    std_dx = np.zeros((N,), dtype=np.float32)
    mean_dy = np.zeros((N,), dtype=np.float32)
    std_dy = np.zeros((N,), dtype=np.float32)

    # Compute offsets for each point i across all noised versions
    for i in range(N):
        x_gt, y_gt = gt_points[i]
        
        # Extract the x,y coords from all noised sets for this index i
        x_noisy = noisy_stack[:, i, 0]  # shape (n_noisy,)
        y_noisy = noisy_stack[:, i, 1]  # shape (n_noisy,)
        
        # Offsets
        dx = x_noisy - x_gt  # shape (n_noisy,)
        dy = y_noisy - y_gt
        
        mean_dx[i] = np.mean(dx)
        std_dx[i]  = np.std(dx)
        mean_dy[i] = np.mean(dy)
        std_dy[i]  = np.std(dy)

    return mean_dx, std_dx, mean_dy, std_dy

def generate_noisy_trajectory(gt_points, mean_dx, std_dx, mean_dy, std_dy):
    """
    Generate a new noisy trajectory by sampling per-point offsets from
    N(mean_dx[i], std_dx[i]^2) and N(mean_dy[i], std_dy[i]^2).
    Returns an array shape (N, 2).
    """
    gt_points = np.asarray(gt_points)
    N = gt_points.shape[0]
    new_noisy = np.zeros((N, 2), dtype=np.float32)
    
    for i in range(N):
        x_gt, y_gt = gt_points[i]
        
        # Sample from Normal distribution for dx, dy
        dx_i = np.random.normal(mean_dx[i], std_dx[i]) if std_dx[i] > 0 else mean_dx[i]
        dy_i = np.random.normal(mean_dy[i], std_dy[i]) if std_dy[i] > 0 else mean_dy[i]
        
        # Create new point
        new_x = x_gt + dx_i
        new_y = y_gt + dy_i
        
        new_noisy[i] = (new_x, new_y)
    
    return new_noisy
import os
def list_immediate_subdirs(base_dir):
    """
    Lists only immediate subdirectories (one level deep) in the given directory.
    """
    return [os.path.join(base_dir, d) for d in os.listdir(base_dir) 
            if os.path.isdir(os.path.join(base_dir, d))]

import pandas as pd
    # Suppose we have a ground-truth trajectory of length N=5 (just for demonstration)
gt_points = pd.read_csv("/home/ibu/bitirme_ws/no_noise/150_point_loc_1_0_noise_0_0/data/sampled_drone_pixel.csv")
gt_points=np.array(gt_points)
noised_points_list = []

noise_dir="/home/ibu/bitirme_ws/extraxt_noise"
input_dirs=list_immediate_subdirs(noise_dir)
for dir in input_dirs:
    if "toplama_csv" in dir:
        continue
    base_name=os.path.basename(dir)
    # print(base_name)
    data_dir=os.path.join(dir,"data")
    if os.path.isdir(data_dir):
        sampled_csv_path=os.path.join(data_dir,"sampled_drone_pixel.csv")
        sampled_points=pd.read_csv(sampled_csv_path)
        sampled_points_np=np.array(sampled_points)
        noised_points_list.append(sampled_points_np)



# 1) Fit the noise model
mean_dx, std_dx, mean_dy, std_dy = fit_pointwise_noise_model(gt_points, noised_points_list)

print("Per-point mean_dx:", mean_dx)
print("Per-point std_dx: ", std_dx)
print("Per-point mean_dy:", mean_dy)
print("Per-point std_dy: ", std_dy)

# 2) Generate new noisy trajectory from the fitted distribution
new_noisy_trajectory = generate_noisy_trajectory(gt_points, mean_dx, std_dx, mean_dy, std_dy)

print("\nGenerated new noisy trajectory:")
print(new_noisy_trajectory)

import cv2 as cv

blank_image=np.zeros((480,640))

for resamp in new_noisy_trajectory:
    print(resamp)
    x=int(resamp[0])
    y=int(resamp[1])

    cv.circle(blank_image,(x,y),1,(255,0,255),2)

cv.imshow("balnk",blank_image)
cv.waitKey(0)


