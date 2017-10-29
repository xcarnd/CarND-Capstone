from keras.models import Model
from keras.layers import Input
from keras.layers.core import Dense, Flatten, Lambda, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.models import load_model
from keras import initializers

def get_model(weights_file='detector_weights.h5'):
    inputs = Input([224, 224, 3])
    x = Lambda(lambda x : (x / 255.0) - 0.5)(inputs)
    x = Conv2D(24, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = MaxPooling2D(padding='SAME')(x)
    x = Dropout(0.5)(x)
    x = Conv2D(36, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = MaxPooling2D(padding='SAME')(x)
    x = Dropout(0.5)(x)
    x = Conv2D(48, (5, 5), strides = (2, 2), activation='relu', padding='SAME')(x)
    x = MaxPooling2D(padding='SAME')(x)
    x = Dropout(0.5)(x)
    x = Conv2D(64, (3, 3), activation='relu', padding='SAME')(x)
    x = Flatten()(x)
    x = Dense(300, activation='relu')(x)
    x = Dropout(0.5)(x)
    x = Dense(50, activation='relu')(x)
    x = Dropout(0.5)(x)
    x = Dense(10)(x)
    pred = Dense(3, activation='softmax')(x)

    model = Model(inputs, pred)
    model.load_weights(weights_file)
    return model
