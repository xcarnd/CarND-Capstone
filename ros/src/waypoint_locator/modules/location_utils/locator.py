#!/usr/bin/env python

from styx_msgs.msg import Lane, Waypoint
import tf
import numpy as np
import math
from scipy.spatial import cKDTree

class WaypointLocator(object):
    def __init__(self, base_waypoints):
        self.all_ref_waypoints = None
        # store the frenet coordinate of the waypoints
        self.waypoints_frenet_s = []
        # length of the track
        self.length_of_track = 0
        self.kdtree = None

        all_ref_waypoints = []
        waypoints = base_waypoints
        xy_pairs = []
        previous_waypoint = None
        s = []
        total_length = 0
        for waypoint in base_waypoints:
            all_ref_waypoints.append(waypoint)
            xy_pairs.append((waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            if previous_waypoint is not None:
                delta_dist = self.dist2d(previous_waypoint, waypoint)
                total_length += delta_dist
            s.append(total_length)
            previous_waypoint = waypoint
            
        self.waypoints_frenet_s = s
        self.length_of_track = total_length
                
        self.kdtree = cKDTree(xy_pairs)
        self.all_ref_waypoints = all_ref_waypoints

    def locate_waypoints_around(self, pose):
        # use kdtree for looking up the closest waypoint
        # the closest waypoint may be either behind or ahead of the querying pose
        # so looking up one waypoint back and one waypoint forward and
        # determine.
        num_all_ref_waypoints = len(self.all_ref_waypoints)

        _, closest_index = self.kdtree.query((pose.position.x, pose.position.y))

        i, j, k = closest_index - 1, closest_index, closest_index + 1
        if i < 0:
            i += num_all_ref_waypoints
        if k >= num_all_ref_waypoints:
            k -= num_all_ref_waypoints 

        v_j_i = (self.all_ref_waypoints[i].pose.pose.position.x - self.all_ref_waypoints[j].pose.pose.position.x,
                 self.all_ref_waypoints[i].pose.pose.position.y - self.all_ref_waypoints[j].pose.pose.position.y)
        v_j_k = (self.all_ref_waypoints[k].pose.pose.position.x - self.all_ref_waypoints[j].pose.pose.position.x,
                 self.all_ref_waypoints[k].pose.pose.position.y - self.all_ref_waypoints[j].pose.pose.position.y)
        v_j_p = (pose.position.x - self.all_ref_waypoints[j].pose.pose.position.x,
                 pose.position.y - self.all_ref_waypoints[j].pose.pose.position.y)
        ca1 = self.get_cos_angle_between(v_j_p, v_j_i)
        ca2 = self.get_cos_angle_between(v_j_p, v_j_k)

        # FIXME: to determin which waypoints is ahead which one behind,
        # the orientation of the pose is needed.
        nearest, behind, ahead = (ca1 > ca2) and (j, i, j) or (j, j, k)
        return nearest, behind, ahead

    def get_cos_angle_between(self, v1, v2):
        product = np.linalg.norm(v1) * np.linalg.norm(v2)
        return 1 if product == 0 else np.dot(v1, v2) / product

    def dist2d(self, p1, p2):
        return math.sqrt((p1.pose.pose.position.x - p2.pose.pose.position.x) ** 2 +
                         (p1.pose.pose.position.y - p2.pose.pose.position.y) ** 2)
