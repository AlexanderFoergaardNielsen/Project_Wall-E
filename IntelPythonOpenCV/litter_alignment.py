#!/usr/bin/env python3
import requests, json
import time
import numpy as np
import cv2
import pyrealsense2 as rs

from mir_navigation import MiR


# from Tracking import Track


class ComputerVision():

    def __init__(self):
        return None


class ComputerVision(object):

    def get_litter_center_point(self, center):
        center_point = center
        return center_point

    def get_depth_information(self, depth):
        dis = depth
        return dis

    def cathethus_length(self, depth):

        # find cathethus length
        hypotenuse = depth  # depth measured in meters
        cathetus_height = 0.35  # some measured constant in meters
        cathetus_length = (np.sqrt(
            hypotenuse ** 2 - cathetus_height ** 2)) - (0.20)

        return cathetus_length

    def calculate_orientation_mir(self, depth, center_X, cathetus, mir_orientation):
        fov_x = 69  # degrees in x direction
        resolution_x = 640
        center = center_X
        camera_orientation = float(abs(fov_x * (center / resolution_x) - fov_x / 2))

        mir_origin_to_camera = 0.5  # meters

        # find cathethus length
        hypotenuse = depth  # depth measured in meters
        cathetus_height = 0.35  # some measured constant in meters
        cathetus_length = cathetus  # - some constant c so we don't hit the object

        mir_origin_to_litter_distance = np.sqrt(
            mir_origin_to_camera ** 2 + cathetus_length ** 2 - 2 * mir_origin_to_camera * cathetus_length * np.cos(
                (180 - camera_orientation) * (np.pi / 180)))

        mir_orientation_to_litter = (
                                            cathetus_length ** 2 - mir_origin_to_camera ** 2 - mir_origin_to_litter_distance ** 2) / -(
                2 * mir_origin_to_camera * mir_origin_to_litter_distance)

        mir_orientation_to_litter = np.arccos(mir_orientation_to_litter) * (180 / np.pi)

        mir_current_orientation = mir_orientation

        mir_aligned_orientation = 0

        if center > resolution_x / 2:
            mir_orientation_to_litter = mir_orientation_to_litter * (-1)
            mir_aligned_orientation = mir_current_orientation + mir_orientation_to_litter
            if mir_current_orientation + mir_orientation_to_litter < -180:
                mir_aligned_orientation = 180 - abs(mir_orientation_to_litter)

        elif center < resolution_x / 2:
            mir_aligned_orientation = mir_current_orientation + mir_orientation_to_litter
            if mir_current_orientation + mir_orientation_to_litter > 180:
                mir_aligned_orientation = abs(mir_orientation_to_litter) - 180

        return mir_aligned_orientation
