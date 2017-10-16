#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from waypoint_locator.srv import *

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # LocateWaypointsAround service
        self.locate_waypoints_around = rospy.ServiceProxy('/waypoint_locator/locate_waypoints_around', LocateWaypointsAround)
        rospy.wait_for_service('/waypoint_locator/locate_waypoints_around')

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        req = LocateWaypointsAroundRequest(pose)
        resp = self.locate_waypoints_around(req)
        return resp.nearest

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        # find the closest visible traffic light (if one exists)
        light = None
        # for testing, uncomment below
        # light = True
        if light:
            wp_idx = self.look_for_traffic_light_ahead(car_position, stop_line_positions)
            if wp_idx > -1:
                state = self.get_light_state(light)
                # for testing, comment above line and uncomment below
                # state = TrafficLight.RED
                return wp_idx, state
        return -1, TrafficLight.UNKNOWN

    def look_for_traffic_light_ahead(self, wp_car_pose_idx, stop_line_positions):
        # fine to use brute-force searching since the number of
        # traffic lights are quite small.
        # looking for the closest light position
        if self.waypoints is None:
            return -1
        pose = self.waypoints[wp_car_pose_idx].pose.pose
        min_dist = 1e7
        min_idx = -1
        for idx, light in enumerate(self.lights):
            light_pos = light.pose.pose.position
            dist = self.dist2d(pose.position, light_pos)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        
        # the closest one doesn't mean the one ahead
        # get the stop line position for the light and check
        # if the vehicle has passed the nearest waypoint
        # of the stop line. if yes, then we should head for the
        # next traffic light.
        #
        # but in fact we don't have to seek the light ahead in this
        # case, because the light detector will only detect the
        # nearest light around the vehicle's current position, the
        # next traffic light is still out of sight at this moment.
        x, y = stop_line_positions[min_idx]
        stop_line_pos = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
        wp_stop_line_idx = self.get_closest_waypoint(stop_line_pos)
        if wp_car_pose_idx > wp_stop_line_idx:
            return -1
        else:
            return wp_stop_line_idx

    def dist2d(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
