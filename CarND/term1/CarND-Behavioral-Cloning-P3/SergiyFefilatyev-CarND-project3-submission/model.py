import csv
import cv2
import numpy as np
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
import sklearn
import random
import matplotlib.pyplot as plt

CORRECTION = 0.1 # the value of correction for steering between side and central cemeras

samples = []
with open('data/driving_log.csv') as file:
    reader = csv.reader(file)
    next(reader, None) # skip header
    for line in reader:
        samples.append(line)

def generator(samples, batch_size=32):
    """Generator for batches for training and validation."""
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        random.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = [] # images
            measurements = [] # steering angles
            for batch_sample in batch_samples:
                    for i in range(3): # iterating through center, left, and right cameras
                        source_path = batch_sample[i]
                        filename = source_path.split('/')[-1]
                        current_path = './data/IMG/' + filename
                        image = cv2.imread(current_path)
                        # OpenCV's opens files in BGR, need to correct
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        images.append(image)
                        measurement = float(batch_sample[3])
                        if i==0: # central camera
                            pass
                        elif i==1: # left camera
                            measurement+=CORRECTION
                        elif i==2: # right camera
                            measurement-=CORRECTION
                        measurements.append(measurement)
                        images.append(cv2.flip(image,1)) # augmenting with a horizontal flip
                        measurements.append(measurement*-1.0)

            X_train = np.array(images)
            y_train = np.array(measurements)
            yield sklearn.utils.shuffle(X_train, y_train)

from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

batch_size = 32

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)

# Using an NVIDIA model with five convolutional layers and four fully connected
# see paper: http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
# Addition to NVIDIA - added a drop-out layer for regularization
model = Sequential()
model.add(Lambda(lambda x: ( x / 255.0 ) - 0.5, input_shape=(160, 320, 3)))
model.add(Cropping2D(cropping=((70,25), (0,0))))
model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100))
model.add(Dropout(0.5))
model.add(Dense(50))
model.add(Dropout(0.5))
model.add(Dense(10))
model.add(Dense(1))

# Using Adam optimizer allows us not to pick learning rates
model.compile(loss='mse', optimizer='adam') 
model.fit_generator(train_generator, samples_per_epoch= \
            len(train_samples) / batch_size , validation_data=validation_generator, \
            nb_val_samples=len(validation_samples) / batch_size , nb_epoch=5)
# saving model in hdf format
model.save('model.h5')

