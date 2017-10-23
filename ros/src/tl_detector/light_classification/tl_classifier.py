import rospy
import rospkg

from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import numpy as np
import time

from keras.models import load_model
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # I've trained a MobileNet for doing inference in the
        # simulator. Not sure it will work well when running
        # in Carla
        rospy.loginfo("Loading model...")
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('tl_detector') + '/light_classification'
        self.model = load_model(base_path + '/simulator-cnn.h5')
        self.signal_indices = [
            TrafficLight.GREEN,
            TrafficLight.YELLOW,
            TrafficLight.RED ]
        
        rospy.loginfo("Warming up classifier by inferencing a sample. (Which shall be detected as a green light, of which ID is 2)")
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
        # crop and resize the image to 224 x 224
        cropped = image[0:600, 100:700]
        resized = cv2.resize(cropped, (224, 224))

        # predict
        start = time.time()
        pred = self.model.predict(resized[None, :, :, :], batch_size=1)
        idx = np.argmax(pred)
        used = time.time() - start
        
        return self.signal_indices[idx]
