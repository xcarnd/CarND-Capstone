#!/usr/bin/env python

import argparse
import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Model
from keras.layers import Input, BatchNormalization
from keras.layers.core import Dense, Flatten, Lambda, Activation, Dropout
from keras.layers.convolutional import Conv2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from keras.models import load_model
from keras import initializers
from keras.preprocessing.image import ImageDataGenerator
from keras.applications.mobilenet import MobileNet
from keras.applications.resnet50 import ResNet50
import keras.callbacks
import math
import pickle
import os
import random
import glob

parser = argparse.ArgumentParser(description = "Model trainer")
parser.add_argument('--model', default="model", metavar='MODEL', help='name of the output model.')
parser.add_argument('--epochs', default="10", type=int, metavar='EPOCHS', help='number of epochs to train.')
parser.add_argument('--batch-size', default="20", type=int, metavar='BSIZE', help='batch size.')
args = parser.parse_args()

# hyper parameters
# batch size
batch_size = 20
if args.batch_size:
    batch_size = args.batch_size

input_size = (224, 224)

TRAIN_DIR="samples/aug"

def load_data(direct):
    dirs = os.listdir(direct)
    dirs = [os.path.join(direct, d) for d in dirs]
    samples = []
    labels = []
    for d in dirs:
        dir_name = os.path.basename(d)
        if dir_name == 'yellow':
            label = np.array([0.0, 1.0, 0.0])
        elif dir_name == 'green':
            label = np.array([1.0, 0.0, 0.0])
        elif dir_name == 'red':
            label = np.array([0.0, 0.0, 1.0])
        else:
            pass
        
        for f in glob.glob(os.path.join(d, "*.jpg")):
            img = cv2.imread(f)
            samples.append(img)
            labels.append(label)
    return np.array(samples), np.array(labels)

datagen = ImageDataGenerator(
    samplewise_center=True)

X, y = load_data(TRAIN_DIR)
print("Shape of X: {}".format(X.shape))
print("Shape of y: {}".format(y.shape))

model_name = "model"
if args.model:
    model_name = args.model

# model definition - NVidia architecture
tensor_input_shape = input_size + (3, )
inputs = Input(shape=tensor_input_shape)

# use mobilenet
# model = ResNet50(input_shape=tensor_input_shape,
#                  include_top=False, 
#                  input_tensor=inputs,
#                  pooling='max')

model = MobileNet(input_shape=tensor_input_shape,
                  dropout=0.5,
                  include_top=False, 
                  input_tensor=inputs,
                  pooling='max')

# freeze the network
#for layer in model.layers:
#    layer.trainable = False

x = model.output
pred = Dense(3, activation='softmax')(x)

model = Model(model.input, pred)
model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

tb_cb = keras.callbacks.TensorBoard(log_dir='tensorboard-log', batch_size=batch_size)
cbks = [tb_cb]
    
# train model
print("Start training model {}".format(model_name))

history = model.fit(x=X, y=y,
                    batch_size=batch_size, epochs=args.epochs,
                    validation_split=0.12,
                    shuffle=True,
                    callbacks=cbks)

# save model
model.save(model_name + ".h5")
print("Model saved to {}".format(model_name + ".h5"))

print(history.history)
