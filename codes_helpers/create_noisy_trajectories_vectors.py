import numpy as np

def trajectory_to_dist_angle(points):
    """
    Convert a trajectory of (x, y) points into a list of (distance, angle) 
    pairs for consecutive segments.
    
    Args:
        points: shape (N, 2) => [(x0, y0), (x1, y1), ..., (x_{N-1}, y_{N-1})]
    Returns:
        dist_angle: shape (N-1, 2),
                    dist_angle[i] = (d_i, theta_i),
                    d_i = distance between points i and i+1,
                    theta_i = angle in radians for segment i (atan2(dy, dx))
    """
    points = np.asarray(points)
    N = len(points)
    if N < 2:
        raise ValueError("Need at least 2 points to compute distance & angle.")
    
    dist_angle = np.zeros((N-1, 2), dtype=np.float32)
    
    for i in range(N-1):
        x1, y1 = points[i]
        x2, y2 = points[i+1]
        dx = x2 - x1
        dy = y2 - y1
        dist = np.sqrt(dx*dx + dy*dy)
        angle = np.arctan2(dy, dx)
        print(f"i{i} dist{dist}, angle{angle}")
        dist_angle[i] = [dist, angle]
    
    return dist_angle
def dist_angle_to_trajectory(dist_angle, start_xy=(0.0, 0.0)):
    """
    Reconstruct (x, y) coordinates from distance–angle pairs.
    
    Args:
        dist_angle: shape (M, 2) => [(d0, th0), (d1, th1), ..., (d_{M-1}, th_{M-1})]
        start_xy:   the (x0, y0) to begin from. 
                    Typically the clean trajectory's first point if you're 
                    replicating the same global position, or (0,0) if you only 
                    care about relative shape.

    Returns:
        points: shape (M+1, 2) => the reconstructed trajectory.
    """
    M = len(dist_angle)
    points = np.zeros((M+1, 2), dtype=np.float32)
    points[0] = start_xy
    
    for i in range(M):
        d_i, th_i = dist_angle[i]
        x_prev, y_prev = points[i]
        x_new = x_prev + d_i * np.cos(th_i)
        y_new = y_prev + d_i * np.sin(th_i)
        points[i+1] = [x_new, y_new]
    
    return points
def fit_dist_angle_noise(clean_pts, noised_pts_list):
    """
    1) Convert the clean trajectory to dist-angle => shape (N-1, 2)
    2) Convert each noised trajectory to dist-angle => shape (N-1, 2)
    3) For each segment i in [0..N-2], compute the difference:
         dd_i = (dist_i^noisy - dist_i^clean)
         da_i = angle_i^noisy - angle_i^clean
       across all noised examples, then fit mean & std.

    Returns:
      mean_dd, std_dd, mean_da, std_da => each shape (N-1,)
    """
    clean_da = trajectory_to_dist_angle(clean_pts)  # shape (N-1, 2)
    # clean_da[i] = (distC_i, angleC_i)
    
    n_noisy = len(noised_pts_list)
    # We'll assume each noisy has shape (N,2) with same N
    # Convert them
    noised_da_list = [trajectory_to_dist_angle(npts) for npts in noised_pts_list]
    # Each noised_da has shape (N-1, 2)
    
    N_minus_1 = clean_da.shape[0]
    # We'll build arrays for dd & da for each segment i across all noised examples
    # shape => (n_noisy, N-1)
    all_dd = np.zeros((n_noisy, N_minus_1), dtype=np.float32)
    all_da = np.zeros((n_noisy, N_minus_1), dtype=np.float32)
    
    for i in range(n_noisy):
        # Dist-Angle for this noised example
        nd = noised_da_list[i]
        # differences
        dd_i = nd[:, 0] - clean_da[:, 0]  # distNoisy_i - distClean_i
        da_i = nd[:, 1] - clean_da[:, 1]  # angleNoisy_i - angleClean_i
        
        all_dd[i] = dd_i
        all_da[i] = da_i

    # Now compute mean & std across noised examples, dimension 0
    mean_dd = np.mean(all_dd, axis=0)  # shape (N-1,)
    std_dd  = np.std(all_dd, axis=0)
    mean_da = np.mean(all_da, axis=0)
    std_da  = np.std(all_da, axis=0)

    return mean_dd, std_dd, mean_da, std_da
def create_noisy_dist_angle(clean_pts, mean_dd, std_dd, mean_da, std_da):
    """
    Generate a new distance–angle sequence by adding random
    noise to the clean dist-angle representation.

    Args:
      clean_pts: shape (N, 2)
      mean_dd, std_dd, mean_da, std_da: shape (N-1,)

    Returns:
      new_dist_angle: shape (N-1, 2) => [ (dist_i_noisy, angle_i_noisy) ]
    """
    clean_da = trajectory_to_dist_angle(clean_pts)  # (N-1, 2)
    N_minus_1 = clean_da.shape[0]

    # We'll produce a new dist-angle for each segment i
    new_da = np.zeros((N_minus_1, 2), dtype=np.float32)

    for i in range(N_minus_1):
        distC_i, angleC_i = clean_da[i]
        # Sample noise
        dd_i = np.random.normal(mean_dd[i], std_dd[i]) if std_dd[i] > 0 else mean_dd[i]
        da_i = np.random.normal(mean_da[i], std_da[i]) if std_da[i] > 0 else mean_da[i]

        dist_noisy = distC_i + dd_i
        angle_noisy = angleC_i + da_i

        # You might want to clamp dist_noisy to be >= 0 if negative distances can occur
        if dist_noisy < 0.0:
            dist_noisy = 0.0

        new_da[i] = [dist_noisy, angle_noisy]

    return new_da
def create_noisy_trajectory(clean_pts, mean_dd, std_dd, mean_da, std_da):
    """
    Full pipeline:
     1) create new noisy dist-angle
     2) reconstruct (x,y) from it, starting from the original first point
    """
    # 1) create new dist-angle
    new_da = create_noisy_dist_angle(clean_pts, mean_dd, std_dd, mean_da, std_da)
    # 2) recon to (x,y)
    start_xy = clean_pts[0]  # or (0,0) if you prefer
    new_xy = dist_angle_to_trajectory(new_da, start_xy=start_xy)
    return new_xy
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
gt_points = np.vstack([gt_points, gt_points[0]])

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
        sampled_points_np=np.vstack([sampled_points_np,sampled_points_np[0]])
        noised_points_list.append(sampled_points_np)


mean_dd, std_dd, mean_da, std_da = fit_dist_angle_noise(gt_points, noised_points_list)

print("mean_dd:", mean_dd)
print("std_dd: ", std_dd)
print("mean_da:", mean_da)
print("std_da: ", std_da)
import matplotlib.pyplot as plt

# Plot ground truth
plt.plot(gt_points[:, 0], gt_points[:, 1], label="Ground Truth", color="green")

# Plot noisy trajectories
for i, pts in enumerate(noised_points_list):
    plt.plot(pts[:, 0], pts[:, 1], label=f"Noisy {i+1}", alpha=0.5)

# Plot generated noisy trajectory

plt.legend()
plt.title("Trajectory Comparison")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid()
plt.show()
# 2) Generate a new "noisy" trajectory
import cv2 as cv
while True:
    new_noisy_xy = create_noisy_trajectory(gt_points, mean_dd, std_dd, mean_da, std_da)
    print("\nNew noisy trajectory:\n", new_noisy_xy)


    blank_image=np.zeros((480,640))
    import cv2 as cv
    for resamp in new_noisy_xy:
        # print(resamp)
        x=int(resamp[0])
        y=int(resamp[1])

        cv.circle(blank_image,(x,y),1,(255,0,255),2)

    cv.imshow("balnk",blank_image)
    cv.waitKey(0)

