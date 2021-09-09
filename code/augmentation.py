import json
import cv2
import numpy as np
import tensorflow as tf
from matplotlib.image import imread
import matplotlib
import matplotlib.pyplot as plt
from PIL import Image
import os
"""
# Opening JSON file
f = open('data.json',)

# returns JSON object as
# a dictionary
data = json.load(f)

# Iterating through the json
# list
for i in data['emp_details']:
    print(i)

# Closing file
f.close()
"""
for i,filename in enumerate(os.listdir(os.getcwd())):
    # f = os.path.join(directory, filename)
    # print(filename)
    if filename.endswith('jpg'):
        print("Augmenting file:{}".format(i))
        im = imread(filename)
        
        adjusted = tf.image.adjust_jpeg_quality(im, jpeg_quality=17)
        tf.keras.preprocessing.image.save_img("aug_r_{}".format(filename),adjusted)

        adjusted2 = tf.image.adjust_gamma(im,gamma=1.5,gain=0.9)
        tf.keras.preprocessing.image.save_img("aug_g_{}".format(filename),adjusted2)
        
        adjusted3= tf.image.adjust_contrast(im,contrast_factor=1.5)
        tf.keras.preprocessing.image.save_img("aug_c_{}".format(filename),adjusted3)
        
        common_type = tf.float32 # Make noise and image of the same type
        gnoise = tf.random.normal(shape=tf.shape(im), mean=0.0, stddev=0.05, dtype=common_type)
        image_type_converted = tf.image.convert_image_dtype(im, dtype=common_type, saturate=False)
        adjusted4 = tf.add(image_type_converted, gnoise)
        tf.keras.preprocessing.image.save_img("aug_n_{}".format(filename),adjusted4)
