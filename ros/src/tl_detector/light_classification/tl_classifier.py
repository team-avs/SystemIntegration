from styx_msgs.msg import TrafficLight
import os
import numpy as np
import cv2
import math
import sys
import operator
from keras.optimizers import Adam
from keras.models import Sequential, load_model
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.utils.np_utils import to_categorical
from keras.layers.normalization import BatchNormalization
from time import gmtime, strftime

DEBUG_MODE = False
DATA_PATH = os.path.dirname(os.path.realpath(__file__)) + '/data/bag_dump_just_traffic_light'
SCALE = 4
CLASSES = {'red': TrafficLight.RED, 'yellow': TrafficLight.YELLOW, 'green': TrafficLight.GREEN, 'nolight': 3, 'unidentified': TrafficLight.UNKNOWN}

class TLClassifier(object):

    def __init__(self):
        self.batch_size = 10
        self.model = None


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # predict traffic light color
        class_index, probability = self.predict(image)
        print("class: %d, probability: %f" % (class_index, probability))
        return class_index


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
        image = cv2.resize(image, (342, 274)) #(1368/SCALE, 1096/SCALE))

        if DEBUG_MODE:
            cv2.imwrite('tmp/' + strftime("%Y-%m-%d %H:%M:%S", gmtime()) +'.jpg', image)

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
                if len(X) >= batch_size or len(X) >= len(images):
                    yield (np.array(X), to_categorical(np.array(Y), len(CLASSES)))
                    X, Y = [], []


    def train(self):
        self.create_model()
        optimizer = Adam(lr=0.00001)
        self.model.compile(optimizer=optimizer, loss='categorical_crossentropy', metrics=['accuracy'])
        images = self.list_images()
        np.random.shuffle(images)
        train_images = images[:int(len(images)*0.8)]
        validation_images = images[int(len(images)*0.8):]
        print("train size: %d, validation size: %d" % (len(train_images), len(validation_images)))
        self.model.fit_generator(self.generate_data(train_images, batch_size=self.batch_size), steps_per_epoch=math.ceil(len(train_images)/float(self.batch_size)), epochs=12)
        self.model.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model.h5'))
        metrics = self.model.evaluate_generator(self.generate_data(validation_images, batch_size=self.batch_size), steps=math.ceil(len(validation_images)/float(self.batch_size)))
        print(metrics)


    def load(self):
        model_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model.h5')
        if not os.path.isfile(model_file):
            print("Can't load model")
            return
        self.model = load_model(model_file)


    def display_metrics(self):
        self.load()
        images = self.list_images()
        np.random.shuffle(images)
        validation_images = images[:10]
        metrics = self.model.evaluate_generator(self.generate_data(validation_images, batch_size=self.batch_size), steps=math.ceil(len(validation_images)/float(self.batch_size)))
        print(metrics)
        for image in validation_images[:10]:
            class_index, probability = self.predict(cv2.imread(image[0]))
            print("class: %d, predicted class: %d, probability: %f, image: %s" % (image[1], class_index, probability, image[0].replace(DATA_PATH, '')))


    def predict(self, image):
        if not self.model:
            self.load()
        predictions = self.model.predict(np.array([self.preprocess(image)]), batch_size=1)[0]
        return max(enumerate(predictions), key=operator.itemgetter(1))


if __name__ == '__main__':
    classifier = TLClassifier()
    if sys.argv[1] == '-t':
        classifier.train()
    elif sys.argv[1] == '-m':
        classifier.display_metrics()
    else:
        print("Usage: use -t for training and -m for current metrics")
