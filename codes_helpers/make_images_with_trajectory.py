import os
import cv2
import numpy as np
import pandas as pd

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

def draw_trajectory_to_drone_image(image_path,drone_pixels):
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create masks for both red ranges
    lower1 = np.array([0, 19, 0])
    upper1 = np.array([19,255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)

    lower2 = np.array([156,19,0])
    upper2 = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower2, upper2)
    
    # Combine masks
    combined_mask = cv2.bitwise_or(mask1, mask2)
    for pixel in drone_pixels:
        cv2.circle(image,pixel,1,(0,0,255),-1)

    # Apply masks to create results
    result1 = cv2.bitwise_and(image, image, mask=mask1)
    result2 = cv2.bitwise_and(image, image, mask=mask2)
    combined_result = cv2.bitwise_and(image, image, mask=combined_mask)
    kernel = np.ones((1,1), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw bounding boxes on original image
    max_area = 0
    largest_box = None
    
    # First pass: find largest area and draw all boxes in blue
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        # Draw all boxes in blue
        # cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
        
        # Keep track of the largest area
        if area > max_area:
            max_area = area
            largest_box = (x, y, w, h)

    if largest_box != None:
        cv2.circle(image, (int(largest_box[0]+(largest_box[2]/2)), largest_box[1]),2, (255, 255, 255), 2)

    base_dir=os.path.dirname(image_path)    
    
    base_dir=os.path.dirname(base_dir)

    # print(base_dir)   
    base_name=os.path.basename(image_path)
    # print(base_name)

    masked_image_dir=os.path.join(base_dir,"drone_with_trajectory_images")
    if not os.path.exists(masked_image_dir):
        os.makedirs(masked_image_dir)
    masked_image_path=os.path.join(masked_image_dir,base_name)
    # cv2.imshow("image",image)
    # cv2.waitKey(0)
    cv2.imwrite(masked_image_path,image)    



def get_drone_image(image_path):
    image = cv2.imread(image_path)
    # print(image_path)
    base_dir=os.path.dirname(image_path)    
    
    base_dir=os.path.dirname(base_dir)

    # print(base_dir)   
    base_name=os.path.basename(image_path)
    # print(base_name)

    masked_image_dir=os.path.join(base_dir,"masked_images")
    if not os.path.exists(masked_image_dir):
        os.makedirs(masked_image_dir)
    masked_image_path=os.path.join(masked_image_dir,base_name)

    
    image_pointed_dir=os.path.join(base_dir,"pointed_images")
    if not os.path.exists(image_pointed_dir):
        os.makedirs(image_pointed_dir)
    image_pointed_path=os.path.join(image_pointed_dir,base_name)

    image_boxed_dir=os.path.join(base_dir,"boxed_images")
    if not os.path.exists(image_boxed_dir):
        os.makedirs(image_boxed_dir)
    image_boxed_path=os.path.join(image_boxed_dir,base_name)
        # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create masks for both red ranges
    lower1 = np.array([0, 19, 0])
    upper1 = np.array([19,255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)

    lower2 = np.array([156,19,0])
    upper2 = np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower2, upper2)
    
    # Combine masks
    combined_mask = cv2.bitwise_or(mask1, mask2)
    
    # Apply masks to create results
    result1 = cv2.bitwise_and(image, image, mask=mask1)
    result2 = cv2.bitwise_and(image, image, mask=mask2)
    combined_result = cv2.bitwise_and(image, image, mask=combined_mask)
    kernel = np.ones((1,1), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw bounding boxes on original image
    max_area = 0
    largest_box = None
    
    # First pass: find largest area and draw all boxes in blue
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        # Draw all boxes in blue
        # cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
        
        # Keep track of the largest area
        if area > max_area:
            max_area = area
            largest_box = (x, y, w, h)
    
    # Draw the largest box in red with thicker line
    blank_image=np.zeros_like(image)
    if largest_box is not None:
        x, y, w, h = largest_box
        
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
        # Add area text above the largest box
        # cv2.putText(image, f'Largest Area: {int(max_area)}', (x, y-5), 
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
     
    cv2.imwrite(image_boxed_path,image)
    return (int(largest_box[0]+(largest_box[2]/2)),largest_box[1])
def list_immediate_subdirs(base_dir):
    """
    Lists only immediate subdirectories (one level deep) in the given directory.
    """
    return [os.path.join(base_dir, d) for d in os.listdir(base_dir) 
            if os.path.isdir(os.path.join(base_dir, d))]
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
        # Calculate Q1, Q3, and IQR for each axis
        Q1 = df.quantile(0.25)
        Q3 = df.quantile(0.75)
        IQR = Q3 - Q1
        
        # Define the outlier range
        lower_bound = Q1 - threshold * IQR
        upper_bound = Q3 + threshold * IQR
        
        # Filter points within the range
        mask = ((df >= lower_bound) & (df <= upper_bound)).all(axis=1)
    elif method == "zscore":
        # Calculate Z-scores for each axis
        z_scores = (df - df.mean()) / df.std()
        
        # Filter points with Z-scores below the threshold
        mask = (z_scores.abs() < threshold).all(axis=1)
    else:
        raise ValueError("Invalid method. Use 'iqr' or 'zscore'.")
    
    # Return the filtered points
    return df[mask].to_numpy()
def get_traj_iamge(root_dir,use_immeditae=True):
    input_dirs=[]
    if use_immeditae:
        input_dirs=list_immediate_subdirs(root_dir)
    else:
        input_dirs.append(root_dir)
    # print(input_dirs)

    for dir in input_dirs:
        if "toplama_csv" in dir:
            continue
        base_name=os.path.basename(dir)
        # print(base_name)
        data_dir=os.path.join(dir,"data")
        if os.path.isdir(data_dir):
            image_dir=os.path.join(data_dir,"images")
            # print(image_dir)
            xyz_path=os.path.join(data_dir,"iris1_XYZ.csv")
            trajectory_df = pd.read_csv(xyz_path)
            trajectory_df = trajectory_df.sort_values(by=['Sec', 'Nanosec']).reset_index(drop=True)
            # print(trajectory_df.head())

            image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
            image_timestamps = []
            for img_file in image_files:
                try:
                    sec, nanosec = img_file.split('.')[0].split('_')
                    image_timestamps.append({'Image': img_file, 'Sec': int(sec), 'Nanosec': int(nanosec)})
                except ValueError:
                    print(f"Skipping invalid image filename: {img_file}")
            
            image_df = pd.DataFrame(image_timestamps)
            image_df = image_df.sort_values(by=['Sec', 'Nanosec']).reset_index(drop=True)
            # print(image_df.head())

            # start_time_sec = trajectory_df.iloc[0]['Sec']
            # end_time_sec = trajectory_df.iloc[-1]['Sec']+1
            start_time = trajectory_df.iloc[0]['Sec'] + trajectory_df.iloc[0]['Nanosec'] * 1e-9
            end_time = trajectory_df.iloc[-1]['Sec'] + trajectory_df.iloc[-1]['Nanosec'] * 1e-9 +1.2

            first_image=None
            drone_pixels=[]
            
            for _, row in image_df.iterrows():
                current_time = row['Sec'] + row['Nanosec'] * 1e-9
                if start_time <= current_time <= end_time:            
                    image_path=os.path.join(image_dir,row['Image'])

                    if first_image is None:
                        first_image=image_path
                    # print(image_path,os.path.exists(image_path)) 

                    drone_pixels.append(get_drone_image(image_path))


            drone_pixels_outlier=pd.DataFrame(drone_pixels)
            drone_pixels_outlier = drone_pixels_outlier.rename(columns={0: 'x', 1: 'y'})

            drone_pixels_outlier=remove_outliers(drone_pixels_outlier)
            
            resampled_drone_pixels=resample_points(drone_pixels_outlier,num_output=250)
            resampled_drone_pixels_df=pd.DataFrame(resampled_drone_pixels)
            resampled_drone_pixels_df = resampled_drone_pixels_df.rename(columns={0: 'image_x', 1: 'image_y'})


            drone_pixels_csv_path=os.path.join(data_dir,"sampled_drone_pixel.csv")
            resampled_drone_pixels_df.to_csv(drone_pixels_csv_path,index=False)

            drone_pixels_outlier=pd.DataFrame(drone_pixels_outlier)
            drone_pixels_outlier = drone_pixels_outlier.rename(columns={0: 'image_x', 1: 'image_y'})

            drone_pixels_csv_path=os.path.join(data_dir,"drone_pixel.csv")
            drone_pixels_outlier.to_csv(drone_pixels_csv_path,index=False)
            print(drone_pixels_outlier.head())
            for _, row in image_df.iterrows():
                current_time = row['Sec'] + row['Nanosec'] * 1e-9
                if start_time <= current_time <= end_time:            
                    image_path=os.path.join(image_dir,row['Image'])

                    
                    # print(image_path,os.path.exists(image_path)) 

                    draw_trajectory_to_drone_image(image_path,drone_pixels)

            print(len(drone_pixels)) 
            current_image = cv2.imread(first_image)
            current_image2=np.zeros_like(current_image)
            current_image3=np.zeros_like(current_image)
            for pixel in resampled_drone_pixels:
                print(pixel)
                x=int(pixel[0])
                y=int(pixel[1])
                cv2.circle(current_image2,(x,y),1,(0,0,255),2)

            cv2.imshow("sampled",current_image2)
            cv2.waitKey(0)

            # for pixel in drone_pixels_outlier:
            #     print(pixel)
            #     x=int(pixel[0])
            #     y=int(pixel[1])
            #     cv2.circle(current_image3,(x,y),1,(0,0,255),2)

            # cv2.imshow("removed outlires",current_image3)
            # cv2.waitKey(0)
            traj_path=os.path.join(data_dir,"iris1_trajectory_masked.jpg")
            cv2.imwrite(traj_path,current_image2)


get_traj_iamge("/home/ibu/bitirme_ws/traj_data_final_ekleme",use_immeditae=True)