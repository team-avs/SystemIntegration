from styx_msgs.msg import TrafficLight
import os
import numpy as np
import cv2
import tensorflow as tf
from time import gmtime, strftime, time
from timeit import default_timer as timer
import label_map_util


DEBUG_MODE = False
DATA_PATH = os.path.dirname(os.path.realpath(__file__)) + '/data/mixed_dataset'

PATH_TO_CKPT = os.path.dirname(os.path.realpath(__file__)) + '/data/trained_mixed/frozen_inference_graph.pb'
PATH_TO_LABELS = os.path.dirname(os.path.realpath(__file__)) + '/data/tl_label_map.pbtxt'
NUM_CLASSES = 4
# mapping between classifier class and TrafficLight
CLASSES = {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN, 4: TrafficLight.UNKNOWN}
#CLASSES = {1: 0, 2: 1, 3: 2, 4: 4}

# based on https://github.com/tensorflow/models/blob/master/object_detection/object_detection_tutorial.ipynb
class TLClassifier(object):

    def __init__(self):
        self.load()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # predict traffic light color
        class_index, probability = self.predict(image)
        if class_index:
            print("class: %d, probability: %f" % (class_index, probability))
        return class_index


    def list_images(self):
        images = []
        for path in os.listdir(DATA_PATH):
            for filename in os.listdir(os.path.join(DATA_PATH, path)):
                if filename.endswith('.jpg'):
                    images.append(os.path.join(DATA_PATH, path, filename))
        return images


    def load(self):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.detection_graph = tf.Graph()
        with tf.Session(graph=self.detection_graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)


    def display_samples(self):
        images = self.list_images()
        np.random.shuffle(images)
        validation_images = images[:10]

        # Definite input and output Tensors for detection_graph
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        for validation_image in validation_images:
            image_np = cv2.imread(validation_image)
            image_np = cv2.resize(image_np, (200, 150))
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB) # Fix colorspace
            image_np_expanded = np.expand_dims(image_np, axis=0)
            # Actual detection.
            (boxes, scores, classes) = self.session.run(
                [detection_boxes, detection_scores, detection_classes],
                feed_dict={image_tensor: image_np_expanded})

            print(validation_image)
            self.predict(cv2.imread(validation_image))
            #self.show_result_image(image_np, np.squeeze(scores), np.squeeze(boxes), np.squeeze(classes))


    def show_result_image(self, image_np, scores, boxes, classes):
        # Visualization of the results of a detection.
        import visualization_utils as vis_util
        from matplotlib import pyplot as plt
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np, boxes, classes.astype(np.int32),
            scores, self.category_index, use_normalized_coordinates=True,
            line_thickness=8)
        plt.figure(figsize=(12, 8))
        plt.imshow(image_np)
        plt.show()


    def predict(self, image_np, min_score_thresh=0.5):
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        image_np = cv2.resize(image_np, (200, 150))
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        start = timer()
        (boxes, scores, classes) = self.session.run(
                [detection_boxes, detection_scores, detection_classes],
                feed_dict={image_tensor: np.expand_dims(image_np, axis=0)})
        end = timer()

        print("detection time: %f" % (end - start))

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

        # self.show_result_image(image_np, scores, boxes, classes)

        for i, box in enumerate(boxes):
            if scores[i] > min_score_thresh:
                light_class = CLASSES[classes[i]]
                print("DETECTED: %d" % light_class)
                return light_class, scores[i]
        print("Traffic light not detected")
        return None, None


if __name__ == '__main__':
    classifier = TLClassifier()
    classifier.display_samples()
