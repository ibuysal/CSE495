from tensorflow.keras import Input, Model
from tensorflow.keras.layers import Dense, Flatten, Conv2D, MaxPooling2D, Concatenate
from sklearn.model_selection import train_test_split
from tensorflow.keras import Input, Model
from tensorflow.keras.layers import Dense, Flatten, Conv2D, MaxPooling2D, Concatenate, Conv1D, GlobalAveragePooling1D
from tensorflow.keras.layers import Input, Conv1D, GlobalAveragePooling1D, Dense, Lambda, Concatenate
import tensorflow as tf



import matplotlib.pyplot as plt 

im_pointed = tf.keras.preprocessing.image.load_img("/home/ibu/bitirme_ws/datas_20_30/150_points_loc-2_0_20_30_0/data/images/211_700000000.jpg", target_size=(240, 320))
im_pointed_array = tf.keras.preprocessing.image.img_to_array(im_pointed)
im_pointed_array = im_pointed_array / 255.0
plt.imshow(im_pointed_array)  # Replace 0 with the index of the image you want to visualize
plt.axis('off')  # Turn off axis labels for better visualization
plt.title('Image from train_images')  # Optional title
plt.show()