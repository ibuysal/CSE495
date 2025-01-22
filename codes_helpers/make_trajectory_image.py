import os
import cv2
import numpy as np
import pandas as pd


def get_drone_image(image_path):
    image = cv2.imread(image_path)
    print(image_path)
    base_dir=os.path.dirname(image_path)    
    
    base_dir=os.path.dirname(base_dir)

    print(base_dir)   
    base_name=os.path.basename(image_path)
    print(base_name)


    masked_image_dir=os.path.join(base_dir,"masked_images")
    if not os.path.exists(masked_image_dir):
        os.makedirs(masked_image_dir)
    masked_image_path=os.path.join(masked_image_dir,base_name)

    image_pointed_dir=os.path.join(base_dir,"pointed_images")
    if not os.path.exists(image_pointed_dir):
        os.makedirs(image_pointed_dir)
    image_pointed_path=os.path.join(image_pointed_dir,base_name)
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
        cv2.circle(blank_image, (int(largest_box[0]+(largest_box[2]/2)), y),1, (0, 0, 255), 2)
        cv2.circle(image, (int(largest_box[0]+(largest_box[2]/2)), y),1, (0, 0, 255), 2)

        # Add area text above the largest box
        # cv2.putText(image, f'Largest Area: {int(max_area)}', (x, y-5), 
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
   
    cv2.imwrite(masked_image_path,blank_image)    
    cv2.imwrite(image_pointed_path,image)    

    return (int(largest_box[0]+(largest_box[2]/2)),largest_box[1])
def list_immediate_subdirs(base_dir):
    """
    Lists only immediate subdirectories (one level deep) in the given directory.
    """
    return [os.path.join(base_dir, d) for d in os.listdir(base_dir) 
            if os.path.isdir(os.path.join(base_dir, d))]

def get_traj_iamge(root_dir):

    input_dirs=list_immediate_subdirs(root_dir)
    print(input_dirs)

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

            start_time_sec = trajectory_df.iloc[0]['Sec']
            end_time_sec = trajectory_df.iloc[-1]['Sec']+1
            first_image=None
            drone_pixels=[]
            for _, row in image_df.iterrows():
                if start_time_sec <= row['Sec'] <= end_time_sec:                
                    image_path=os.path.join(image_dir,row['Image'])

                    if first_image is None:
                        first_image=image_path
                    # print(image_path,os.path.exists(image_path)) 

                    drone_pixels.append(get_drone_image(image_path))
            current_image = cv2.imread(first_image)
            current_image2=np.zeros_like(current_image)
            for pixel in drone_pixels:
                cv2.circle(current_image2,pixel,1,(0,0,255),2)

            cv2.imshow("output",current_image2)
            cv2.waitKey(0)
            traj_path=os.path.join(data_dir,"iris1_trajectory_masked.jpg")
            cv2.imwrite(traj_path,current_image2)


get_traj_iamge("/home/ibu/bitirme_ws/traj_data_final")