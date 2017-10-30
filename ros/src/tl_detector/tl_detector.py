#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight, TrafficLightState
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import math
from location_utils.locator import WaypointLocator

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.use_ground_truth = rospy.get_param("~use_ground_truth", False)
        if self.use_ground_truth:
            rospy.loginfo("~use_ground_truth is set to true. Classifier will not be invoked. Ground truth value will be used.")
        else:
            rospy.loginfo("~use_ground_truth is set to false. Will invoke classifier to do classification.")
        self.model_type = rospy.get_param("~model_type", "styx")

        self.pose = None
        self.start_detecting_pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

        self.locator = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLightState, queue_size=1)

        self.debug_image_pub = rospy.Publisher('/image_debug', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.model_type)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.reaching_next_traffic_light = True
        self.fov_x = 180 # no limit by default
        self.angle_to_nearest_traffic_light = None
        self.detection_started = False

        self.sub_cam_info = rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_cb, queue_size=1)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        rospy.spin()

    def camera_info_cb(self, msg):
        # calculate fov in x direction
        # tan(fov_x/2) = (image_width / 2) / focal_length_x
        # =>
        # fov_x = atan(image_width / (2 * focal_length_x)) * 2
        image_width = 1368 # this is the dimension of x of ouput from /image_raw
        focal_length_x = msg.K[0]
        fov_x = math.atan(image_width / (2 * focal_length_x)) * 2
        self.fov_x = fov_x * 180 / math.pi
        rospy.loginfo("FOV x (in degrees): {}".format(self.fov_x))
        self.sub_cam_info.unregister()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.locator = WaypointLocator(self.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def publish_debug_image(self):
        if self.camera_image is None:
            return
        # convert image to opencv handlable format
        image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # write debug message into the image
        if self.state == TrafficLight.UNKNOWN:
            debug_text_traffic_light = "Traffic light detection off."
            color = (255, 255, 255)
        elif self.state == TrafficLight.RED:
            debug_text_traffic_light = "Traffic light: RED."
            color = (0, 0, 255)
        elif self.state == TrafficLight.YELLOW:
            debug_text_traffic_light = "Traffic light: YELLOW."
            color = (0, 255, 255)
        elif self.state == TrafficLight.GREEN:
            debug_text_traffic_light = "Traffic light: GREEN."
            color = (0, 255, 0)

        cv2.rectangle(image, (0, 0), (1024, 32 * 4 + 10), (0, 0, 0), cv2.FILLED)
        cv2.putText(image, debug_text_traffic_light, (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, color, 2)
        cv2.putText(image,
                    "Detection started? {}".format(self.detection_started),
                    (16, 32 * 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, color, 2)
        if self.last_wp > -1:
            cv2.putText(image,
                        "Stop line wp loc: ({:.2f}, {:.2f}), current pose: ({:.2f}, {:.2f})".format(
                            self.waypoints[self.last_wp].pose.pose.position.x,
                            self.waypoints[self.last_wp].pose.pose.position.y,
                            self.pose.pose.position.x,
                            self.pose.pose.position.y 
                        ),
                        (16, 32 * 3),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, color, 2)
                    
        if self.angle_to_nearest_traffic_light is not None:
            cv2.putText(image,
                        "Angle to nearest traffic light: {:.2f} (FOVx/2={:.2f})".format(
                            self.angle_to_nearest_traffic_light, self.fov_x / 2),
                        (16, 32 * 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, color, 2)
        
        image_array = np.asarray(image)
        msg = self.bridge.cv2_to_imgmsg(image_array, encoding='bgr8')
        # publish debug topic
        self.debug_image_pub.publish(msg)

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
            if state == TrafficLight.UNKNOWN:
                light_wp = -1
            self.last_wp = light_wp
            self.upcoming_light_pub.publish(TrafficLightState(light_wp, state))
        else:
            self.upcoming_light_pub.publish(TrafficLightState(self.last_wp, self.last_state))
        self.state_count += 1
        # for debug only
        # self.publish_debug_image()

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.locator is None:
            return -1
        nearest, _, _ = self.locator.locate_waypoints_around(pose)
        return nearest

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
        if self.lights is None:
            return -1, TrafficLight.UNKNOWN
        
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        # find the closest visible traffic light (if one exists)
        wp_idx, light_idx = self.look_for_traffic_light_ahead(car_position,
                                                              stop_line_positions)

        if wp_idx > -1:
            light = self.lights[light_idx]
        # to bypass the traffic light detector, set param
        # /tl_detector/use_ground_truth to True to bypass the traffic
        if light:
            if self.use_ground_truth:
                # ground truth is stored in light.state
                state = light.state
            else:
                state = self.get_light_state(light)
                # rospy.loginfo("Detected traffic light state: {}, nearest waypoint index: {}".format(state, wp_idx))
            return wp_idx, state
        return -1, TrafficLight.UNKNOWN

    def look_for_traffic_light_ahead(self, wp_car_pose_idx, stop_line_positions):
        # fine to use brute-force searching since the number of
        # traffic lights are quite small.
        # looking for the closest light position
        self.facing_stop_line = False        
        if self.waypoints is None:
            # rospy.loginfo("Waypoints not loaded yet")
            return -1, -1
        pose = self.waypoints[wp_car_pose_idx].pose.pose
        min_dist = 1e7
        min_idx = -1
        for idx, light in enumerate(self.lights):
            light_pos = light.pose.pose.position
            
            pos = Pose(Point(light_pos.x, light_pos.y, 0), Quaternion(0, 0, 0, 1))
            light_wp_idx = self.get_closest_waypoint(pos)
            
            dist = self.dist2d(pose.position, light_pos)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx

        MAX_EUCLIDEAN_DIST = 100
        # fast filtering out: if the vehicle is still too far away
        # from the nearest traffic light in euclidean distance
        # (specified by MAX_EUCLIDEAN_DIST) , then just report there's
        # no traffic light ahead
        if min_dist > MAX_EUCLIDEAN_DIST:
            # rospy.loginfo("Distance to the nearest red light: {}. Not close enough".format(min_dist))
            return -1, -1

        # we may have passed the nearest light already. to determine
        # whether the vehicle is facing to the light, calculate the
        # angle between the vehicle's heading and the light's facing
        # direction.

        # field of view
        fov_half = self.fov_x / 2
        vehicle_heading = self.get_heading(self.pose.pose)[:2]
        light_pos = self.lights[min_idx].pose.pose.position
        light_from_vehicle = self.make_vector_2d(self.pose.pose.position, light_pos)
        # rospy.loginfo("Vehicle location: {}, heading: {}, direct: {}".format(self.pose.pose.position, vehicle_heading, light_from_vehicle))
        angle = math.acos(
            self.get_cos_angle_2d(vehicle_heading, light_from_vehicle))
        angle = angle * 180 / math.pi
        self.angle_to_nearest_traffic_light = angle
        # rospy.loginfo("angle: {}".format(self.angle_to_nearest_traffic_light))

        if self.reaching_next_traffic_light and angle <= fov_half:
            self.detection_started = True
            self.reaching_next_traffic_light = False
            self.start_detecting_pose = self.pose
            rospy.loginfo("Entered FOV. Detection starts.")
        elif self.detection_started:
            if angle > fov_half:
                self.detection_started = False
                self.start_detecting_pose = None
                rospy.loginfo("Out of FOV. Stop detection.")
            else:
                pass
                # rospy.loginfo("Detection already started")
        elif (not self.detection_started) and angle > fov_half:
            self.reaching_next_traffic_light = True
            # rospy.loginfo("Not entering FOV. Waiting for reaching next traffic light.")
        elif (not self.detection_started) and angle <= fov_half:
            # rospy.loginfo("Waiting for getting out FOV.")
            pass

        if not self.detection_started:
            return -1, -1

        x, y = stop_line_positions[min_idx]
        stop_line_pos = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
        wp_stop_line_idx = self.get_closest_waypoint(stop_line_pos)
        stop_line_wp = self.waypoints[wp_stop_line_idx]

        v_f = self.get_heading(stop_line_wp.pose.pose)
        v_stop_line_heading = [-v_f[1], v_f[0]]
        v_last = self.make_vector_2d(stop_line_wp.pose.pose.position, self.start_detecting_pose.pose.position)
        v_current = self.make_vector_2d(stop_line_wp.pose.pose.position, self.pose.pose.position)
        
        last_side = np.cross(v_stop_line_heading, v_last).take(0)
        current_side = np.cross(v_stop_line_heading, v_current).take(0)
        if last_side * current_side < 0 and abs(last_side - current_side) > 1e-5:
            rospy.loginfo("Vehicle has passed the stop line. Stop detection")
            self.detection_started = False
            self.start_detecting_pose = None
            return -1, -1

        # even if the euclidean distance is close enough, we'll
        # still have to check if they are really close enough by
        # calculating the distance between the two waypoints.
        dist = self.waypoint_distance(wp_car_pose_idx, wp_stop_line_idx)
        # rospy.loginfo("Distance to nearest stop line waypoint: {:.2f}".format(dist))
        if dist > MAX_EUCLIDEAN_DIST:
            return -1, -1

        # rospy.loginfo("Ahead stopline waypoint idx: {}".format(wp_stop_line_idx))
        return wp_stop_line_idx, min_idx

    def make_vector_2d(self, point_from, point_to):
        return [point_to.x - point_from.x, point_to.y - point_from.y]

    def dist2d(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    # calculate the distance between two waypoints by accumulating the
    # length of the piecewise segment. may be integrated into waypoint
    # locator's services.
    def waypoint_distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(self.waypoints[wp1].pose.pose.position, self.waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_heading(self, pose):
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        heading_q = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q, [1, 0, 0, 0]),
            tf.transformations.quaternion_inverse(q))
        return heading_q[:3]

    def get_cos_angle_2d(self, v1, v2):
        product = np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2])
        return 1 if product == 0 else np.dot(v1[:2], v2[:2]) / product

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
