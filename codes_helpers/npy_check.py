import numpy as np

points=np.load("/home/ibu/bitirme_ws/traj_data_final_ekleme_npy/train_observer_start_end.npy")

images=np.load("/home/ibu/bitirme_ws/traj_data_final_ekleme_npy/train_target_start_end.npy")

print(np.shape(points))
print(np.shape(images))