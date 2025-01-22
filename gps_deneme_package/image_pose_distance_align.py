import pandas as pd
from geopy.distance import geodesic
import math
import os

def calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2):
    """
    Calculate 3D distance between two points considering latitude, longitude, and altitude.
    """
    point1 = (lat1, lon1)
    point2 = (lat2, lon2)
    distance_2d = geodesic(point1, point2).meters
    altitude_diff = abs(alt1 - alt2)
    distance_3d = math.sqrt(distance_2d**2 + altitude_diff**2)
    return distance_3d

def list_immediate_subdirs(base_dir):
    """
    Lists only immediate subdirectories (one level deep) in the given directory.
    """
    return [os.path.join(base_dir, d) for d in os.listdir(base_dir) 
            if os.path.isdir(os.path.join(base_dir, d))]

def process_directory(base_dir,output_dir,use_immeditae_dir=True):
    """
    Process the directory and align the pose and navigation data.
    """
    input_dirs=[]
    if use_immeditae_dir:
        input_dirs = list_immediate_subdirs(base_dir)
    else:
        input_dirs.append(base_dir)
    for dir in input_dirs:
        if "toplama_csv" in dir:
            continue
        print(dir)  
        db_name = os.path.basename(dir)
        
        # Setup paths
        image_dir = os.path.join(dir, "data/images")
        pose_iris = os.path.join(dir, "data/iris_pose.csv")
        pose_iris1 = os.path.join(dir, "data/iris1_pose.csv")
        navsat_iris = os.path.join(dir, "data/navsat.csv")
        navsat_iris1 = os.path.join(dir, "data/navsat_1.csv")
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f"{db_name}_distance_navat_pose.csv")
        
        try:
            df1 = pd.read_csv(navsat_iris)
            df2 = pd.read_csv(navsat_iris1)
            
            aligned_distance_df = pd.merge(df1, df2, on=['Sec', 'Nanosec'], suffixes=('_iris', '_iris1'))
            aligned_distance_df['Distance'] = aligned_distance_df.apply(
                lambda row: calculate_distance(
                    row['Latitude_iris'], row['Longitude_iris'], row['Altitude_iris'],
                    row['Latitude_iris1'], row['Longitude_iris1'], row['Altitude_iris1']
                ), axis=1
            )
            
            image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
            image_timestamps = []
            for img_file in image_files:
                try:
                    sec, nanosec = img_file.split('.')[0].split('_')
                    image_timestamps.append({'Image': img_file, 'Sec': int(sec), 'Nanosec': int(nanosec)})
                except ValueError:
                    print(f"Skipping invalid image filename: {img_file}")
            
            image_df = pd.DataFrame(image_timestamps)
            
            # Find closest matches for each image
            def find_closest_row(image_row, csv_df):
                image_sec = image_row['Sec']
                image_nanosec = int(f"{image_row['Nanosec']:09d}")
                csv_df['TimeDifference'] = (
                    abs(csv_df['Sec'] - image_sec) + 
                    abs(csv_df['Nanosec'].astype(int) - image_nanosec) * 1e-9
                )
                return csv_df.loc[csv_df['TimeDifference'].idxmin()]
            
            # Process and match all data
            closest_matches = []
            for _, image_row in image_df.iterrows():
                closest_match = find_closest_row(image_row, aligned_distance_df)
                match_data = {
                    'image': image_row['Image'],
                    'image_sec': image_row['Sec'],
                    'image_nanosec': image_row['Nanosec'],
                    'closest_sec_distance': closest_match['Sec'],
                    'closest_nanosec_distance': closest_match['Nanosec'],
                    'distance': closest_match['Distance'],
                    'time_difference_distance': closest_match['TimeDifference'],
                    'navsat_latitude_iris': closest_match["Latitude_iris"],
                    'navsat_longitude_iris': closest_match["Longitude_iris"],
                    'navsat_altitude_iris': closest_match["Altitude_iris"],
                    'navsat_latitude_iris1': closest_match["Latitude_iris1"],
                    'navsat_longitude_iris1': closest_match["Longitude_iris1"],
                    'navsat_altitude_iris1': closest_match["Altitude_iris1"]
                }
                closest_matches.append(match_data)
            
            # Process pose data
            pose_iris_df = pd.read_csv(pose_iris)
            pose_iris1_df = pd.read_csv(pose_iris1)
            
            for _, image_row in image_df.iterrows():
                closest_match = find_closest_row(image_row, pose_iris_df)
                result = next((item for item in closest_matches if item['image'] == image_row['Image']), None)
                if result:
                    result.update({
                        'x_iris': round(closest_match['Position X'],3),
                        'y_iris': round(closest_match['Position Y'],3),
                        'z_iris': round(closest_match['Position Z'],3),
                        'closest_sec_iris_pose': closest_match['Sec'],
                        'closest_nanosec_iris_pose': closest_match['Nanosec']
                    })
            
            for _, image_row in image_df.iterrows():
                closest_match = find_closest_row(image_row, pose_iris1_df)
                result = next((item for item in closest_matches if item['image'] == image_row['Image']), None)
                if result:
                    result.update({
                        'x_iris1': round(closest_match['Position Y']+5,3),
                        'y_iris1': round(closest_match['Position X'],3),
                        'z_iris1': round(closest_match['Position Z'],3),
                        'closest_sec_iris1_pose': closest_match['Sec'],
                        'closest_nanosec_iris1_pose': closest_match['Nanosec']
                    })
            
            # Save final results
            final_df = pd.DataFrame(closest_matches)
            final_df = final_df.sort_values(by=['image_sec', 'image_nanosec'])
            final_df.to_csv(output_file, index=False)
            
        except Exception as e:
            print(f"Error processing directory {dir}: {str(e)}")
            continue

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        base_dir = sys.argv[1]
        process_directory("/home/ibu/bitirme_ws/traj_data_final_ekleme","/home/ibu/bitirme_ws/traj_data_final_ekleme_csv",True)
    else:
        print("Please provide base directory as argument")