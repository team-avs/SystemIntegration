import os
import numpy as np
import cv2
from keras.optimizers import Adam
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.utils.np_utils import to_categorical
from keras.layers.normalization import BatchNormalization

DATA_PATH = os.path.dirname(os.path.realpath(__file__)) + '/data/bag_dump_just_traffic_light'
SCALE = 4
CLASSES = {'red': 0, 'yellow': 1, 'green': 2, 'nolight': 3, 'unidentified': 4}

class TLDetectorTrain(object):

    def create_model(self):
        self.model = Sequential()

        self.model.add(Conv2D(32, (3, 3), padding='valid', input_shape=(1096/SCALE, 1368/SCALE, 3)))
        self.model.add(MaxPooling2D(pool_size=(2,2)))
        self.model.add(Dropout(0.5))
        self.model.add(Activation('relu'))
        self.model.add(BatchNormalization())

        self.model.add(Flatten())
        self.model.add(Dense(32))
        self.model.add(Activation('relu'))
        self.model.add(Dense(5))
        self.model.add(Activation('softmax'))

    def preprocess(self, image):
        image = cv2.resize(image, (1368/SCALE, 1096/SCALE))
        # image = image / 127.5 - 1 # normalize to -1, 1
        return image

    def list_images(self):
        images = []
        for path in os.listdir(DATA_PATH):
            for filename in os.listdir(os.path.join(DATA_PATH, path)):
                images.append( (os.path.join(DATA_PATH, path, filename), CLASSES[path]) )
        return images


    def generate_data(self, images, batch_size):
        while True:
            X, Y = [], []
            np.random.shuffle(images)
            for image in images:
                img = cv2.imread(image[0])
                X.append(self.preprocess(img))
                Y.append(image[1])
                if len(X) >= batch_size:
                    yield (np.array(X), to_categorical(np.array(Y), len(CLASSES)))
                    X, Y = [], []


    def train(self):
        self.create_model()
        optimizer = Adam(lr=0.00001)
        self.model.compile(optimizer=optimizer, loss='categorical_crossentropy', metrics=['accuracy'])
        batch_size = 10
        images = self.list_images()
        np.random.shuffle(images)
        train_images = images[:int(len(images)*0.8)]
        validation_images = images[int(len(images)*0.8):]
        print("train size: %d, validation size: %d" % (len(train_images), len(validation_images)))
        self.model.fit_generator(self.generate_data(train_images, batch_size=batch_size), steps_per_epoch=len(train_images)/batch_size, epochs=12)
        self.model.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model.h5'))
        metrics = self.model.evaluate_generator(self.generate_data(validation_images, batch_size=batch_size), steps=len(validation_images)/batch_size)
        print(metrics)


if __name__ == '__main__':
    tl_detector_train = TLDetectorTrain()
    tl_detector_train.train()
