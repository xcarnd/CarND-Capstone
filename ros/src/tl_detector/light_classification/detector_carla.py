import numpy as np
from keras.models import Model
from keras.layers import Input
from keras.layers.core import Dense, Flatten, Lambda, Activation, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.models import load_model
import math
import random

input_size = (70, 192)

def get_model(weights):
    inputs = Input(input_size + (3,))
    x = Lambda(lambda x : (x / 255.0) - 0.5)(inputs)
    x = Conv2D(12, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = Conv2D(24, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = Conv2D(36, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = Conv2D(48, (3, 3), activation='relu', padding='SAME')(x)
    x = Conv2D(48, (3, 3), activation='relu', padding='SAME')(x)
    x = Dropout(0.5)(x)
    x = Flatten()(x)
    x = Dense(100, activation='relu')(x)
    x = Dropout(0.5)(x)
    x = Dense(50, activation='relu')(x)
    x = Dropout(0.5)(x)
    x = Dense(10)(x)
    pred = Dense(3, activation='softmax')(x)

    model = Model(inputs, pred)
    model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

    if weights is not None:
        model.load_weights(weights)
    
    return model
