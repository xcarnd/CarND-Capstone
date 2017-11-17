from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import os

class CarlaDetectionModel(object):
    def __init__(self, graph_path):
        self.min_score = 0.3
        
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            serialized_graph = self.load_graph(graph_path)
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
                
            tf_session = tf.Session(graph=detection_graph)
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.tf_session_run = self.make_run_func(tf_session,
                                                     image_tensor,
                                                     detection_scores,
                                                     detection_classes)

    def load_graph(self, graph_path):
        graph_name = os.path.basename(graph_path)
        print("Loading graph from {}".format(graph_path))
        complete_graph = ""
        cnt = 0
        part_file = os.path.join(graph_path, "{}.parts.{}".format(graph_name, cnt))
        while os.path.exists(part_file):
            print("Part file: {}".format(part_file))
            with open(part_file, 'rb') as p:
                complete_graph += p.read()
            cnt += 1
            part_file = os.path.join(graph_path, "{}.parts.{}".format(graph_name, cnt))
        print("Graph loaded")
        return complete_graph
        
    def make_run_func(self, sess, image_tensor, detection_scores, detection_classes):
        def func(image):
            (scores, classes) = sess.run([detection_scores, detection_classes],
                                         feed_dict={image_tensor: image})
            return scores[0], classes[0]
        return func

    def predict(self, image):
        print("Start prediction")
        scores, classes = self.tf_session_run(image)
        if len(scores) == 0:
            return 0
        
        max_score = 0
        idx = None
        for i, score in enumerate(scores):
            if score > self.min_score and score > max_score:
                max_score = score
                idx = i
                
        if idx is None:
            return 0
        return int(classes[idx])

def get_model(graph_path):
    return CarlaDetectionModel(graph_path)
