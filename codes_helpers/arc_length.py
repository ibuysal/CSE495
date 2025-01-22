import numpy as np

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
    # 1) get cumulative arc lengths
    cums = cumulative_arc_lengths(points)
    L = cums[-1]  # total length

    # 2) param steps from 0..1
    new_u = np.linspace(0.0, 1.0, num_output+1)  # e.g. 241 points
    new_points = []

    # 3) for each u, find the corresponding segment in the original points
    for u in new_u:
        target_length = u * L

        # find where cums >= target_length
        idx = np.searchsorted(cums, target_length)
        if idx <= 0:
            # at start
            new_points.append(points[0])
        elif idx >= len(points):
            # at end
            new_points.append(points[-1])
        else:
            # linear interpolation between points[idx-1] and points[idx]
            t = (target_length - cums[idx-1]) / (cums[idx] - cums[idx-1])
            x1, y1 = points[idx-1]
            x2, y2 = points[idx]
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            new_points.append((x, y))

    return new_points

import pandas as pd

pts_clean=pd.read_csv("/home/ibu/bitirme_ws/no_noise/150_point_loc_1_0_noise_0_0/data/drone_pixel.csv")
noisy=pd.read_csv("/home/ibu/bitirme_ws/extraxt_noise/150points_3_3_loc_1_0__noise_40_20/data/drone_pixel.csv")
pts_clean=np.array(pts_clean)
pts_noisy=np.array(noisy)
# Example usage:
# pts_clean = [...]  # (240, 2)
# pts_noisy = [...]  # (290, 2)

resampled_clean = resample_points(pts_clean, num_output=240)  # 241 points
resampled_noisy = resample_points(pts_noisy, num_output=240)  # 241 points

# Now resampled_clean[k] corresponds to resampled_noisy[k] for k=0..240

import cv2 as cv

blank_image=np.zeros((480,640))

for resamp in resampled_clean:
    print(resamp)
    x=int(resamp[0])
    y=int(resamp[1])

    cv.circle(blank_image,(x,y),1,(255,0,255),2)

cv.imshow("balnk",blank_image)

cv.waitKey(0)

blank_image1=np.zeros((480,640))

for resamp in resampled_noisy:
    print(resamp)
    x=int(resamp[0])
    y=int(resamp[1])

    cv.circle(blank_image1,(x,y),1,(255,0,255),2)

cv.imshow("balnk1",blank_image1)

cv.waitKey(0)