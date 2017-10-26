import rospy
import rospkg

from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import numpy as np
import time

import cv2

class TLClassifier(object):
    def __init__(self, model_type):
        self.model_type = model_type
        #TODO load classifier
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('tl_detector') + '/light_classification'
        if model_type == 'styx':
            rospy.loginfo("Loading model for simulator...")
            import detector_model as sim_model
            self.model = sim_model.get_model(base_path + '/detector_weights.h5')
        else:
            rospy.loginfo("Loading model for Carla...")
            import detector_model_carla as carla_model
            self.model = carla_model.get_model(base_path + '/detector_carla_weights.h5')
            
        self.signal_indices = [
            TrafficLight.GREEN,
            TrafficLight.YELLOW,
            TrafficLight.RED ]
        
        rospy.loginfo("Warming up classifier by inferencing a sample. (Which shall be detected as a green light, of which ID is 2. No worry about the wrong answer if model for Carla is loaded)")
        img = cv2.imread(base_path + "/sample.jpg")
        rospy.loginfo("Sample inferenced as signal id: {}".format(self.get_classification(img)))
        start_time = time.time()
        # do inference again and timing
        self.get_classification(img)
        used_time = time.time() - start_time
        rospy.loginfo("Inference on one image takes {} second after warm up.".format(used_time))
        rospy.loginfo("TLClassifier ready")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if self.model_type == 'styx':
            # inputs for simulator are requried to be 224 x 224,
            # crop and resize the image
            cropped = image[0:600, 100:700]
            inp = cv2.resize(cropped, (224, 224))
        else:
            # inputs for Carla are 70 x 192,
            # crop and resize the image
            cropped = image[0:600, 100:700]
            inp = cv2.resize(cropped, (70, 192))

        # predict
        start = time.time()
        pred = self.model.predict(inp[None, :, :, :], batch_size=1)
        idx = np.argmax(pred)
        used = time.time() - start
        
        return self.signal_indices[idx]
