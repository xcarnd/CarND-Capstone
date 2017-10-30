#!/usr/bin/env python

import numpy as np
from scipy.spatial import cKDTree
class WaypointLocator(object):
    def __init__(self, base_waypoints):
        self.kdtree = cKDTree(base_waypoints)
        self.all_ref_waypoints = base_waypoints

    def locate_waypoints_around(self, position):
        num_all_ref_waypoints = len(self.all_ref_waypoints)
        
        dist, closest_index = self.kdtree.query((position[0], position[1]))
        i, j, k = closest_index - 1, closest_index, closest_index + 1
        if i < 0:
            i += num_all_ref_waypoints
        if k >= num_all_ref_waypoints:
            k -= num_all_ref_waypoints 

        v_j_i = (self.all_ref_waypoints[i][0] - self.all_ref_waypoints[j][0],
                 self.all_ref_waypoints[i][1] - self.all_ref_waypoints[j][1])
        v_j_k = (self.all_ref_waypoints[k][0] - self.all_ref_waypoints[j][0],
                 self.all_ref_waypoints[k][1] - self.all_ref_waypoints[j][1])
        v_j_p = (position[0] - self.all_ref_waypoints[j][0],
                 position[1] - self.all_ref_waypoints[j][1])
        ca1 = self.get_cos_angle_between(v_j_p, v_j_i)
        ca2 = self.get_cos_angle_between(v_j_p, v_j_k)
        nearest, behind, ahead = (ca1 > ca2) and (j, i, j) or (j, j, k)
        final_wps = []
        end_idx = ahead + 100
        if end_idx > num_all_ref_waypoints:
            end_idx = num_all_ref_waypoints
        final_wps = self.all_ref_waypoints[ahead:end_idx]
        # print 'ahead:',ahead, position   
        return dist, behind, ahead, final_wps

    def get_cos_angle_between(self, v1, v2):
        product = np.linalg.norm(v1) * np.linalg.norm(v2)
        return 1 if product == 0 else np.dot(v1, v2) / product