import os
import shutil

def find_and_move_db3_files(source_directory, destination_directory):
    """
    Find all .db3 files in the source directory and its subdirectories,
    and move them to the destination directory.

    Args:
        source_directory (str): The root directory to search.
        destination_directory (str): The directory to move .db3 files to.
    """
    if not os.path.exists(destination_directory):
        os.makedirs(destination_directory)

    for root, _, files in os.walk(source_directory):
        for file in files:
            if file.endswith(".db3"):
                source_path = os.path.join(root, file)
                destination_path = os.path.join(destination_directory, file)

                # Move the file
                shutil.move(source_path, destination_path)
                print(f"Moved: {source_path} -> {destination_path}")

# Example usage
source_directory = "/home/ibu/bitirme_ws/traj_data_final_ekleme"  # Replace with your source directory
destination_directory = "/home/ibu/bitirme_ws/traj_data_final_ekleme_bags"  # Replace with your destination directory

find_and_move_db3_files(source_directory, destination_directory)