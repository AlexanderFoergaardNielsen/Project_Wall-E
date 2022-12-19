#!/usr/bin/env python3
import requests, json
import time
import numpy as np
import cv2
import pyrealsense2 as rs

from mir_navigation import MiR
#from Tracking import Track
mir = MiR()

class ComputerVision():

    def __init__(self):
        return None

class ComputerVision(object):


    def get_litter_center_point(self, center):
        center_point= center
        return center_point
    def get_depth_information(self, depth):
        dis = depth
        return dis


    def cathethus_length(self, depth):

        # find cathethus length
        hypotenuse = depth  # depth measured in meters
        cathetus_height = 0.35  # some measured constant in meters
        cathetus_length = (np.sqrt(
            hypotenuse ** 2 - cathetus_height ** 2))

        return cathetus_length

    def calculate_orientation_mir(self, depth, center_X, cathetus, mir_orientation):
        fov_x = 69  # degrees in x direction
        resolution_x = 640
        center = center_X
        print('center: ',center)
        camera_orientation = float(abs(fov_x * (center / resolution_x) - fov_x/2))
        print(camera_orientation)

        mir_origin_to_camera = 0.5  # meters

        # find cathethus length
        hypotenuse = depth# depth measured in meters
        print('Depth: ',depth)
        cathetus_height = 0.35  # some measured constant in meters
        cathetus_length = cathetus  # - some constant c so we don't hit the object

        mir_origin_to_litter_distance = np.sqrt(
            mir_origin_to_camera ** 2 + cathetus_length ** 2 - 2 * mir_origin_to_camera * cathetus_length * np.cos(
                (180 - camera_orientation) * (np.pi / 180)))

        print('z: ', mir_origin_to_litter_distance)

        mir_orientation_to_litter = (
                                                cathetus_length ** 2 - mir_origin_to_camera ** 2 - mir_origin_to_litter_distance ** 2) / -(
                    2 * mir_origin_to_camera * mir_origin_to_litter_distance)

        mir_orientation_to_litter = np.arccos(mir_orientation_to_litter) * (180 / np.pi)

        print('MirLitterOrientation: ', mir_orientation_to_litter)

        mir_current_orientation = mir_orientation

        mir_aligned_orientation = 0

        if center > resolution_x/2:
            mir_orientation_to_litter = mir_orientation_to_litter * (-1)
            mir_aligned_orientation = mir_current_orientation + mir_orientation_to_litter
            print('Bigger >')
            if mir_current_orientation + mir_orientation_to_litter < -180:
                mir_aligned_orientation = 180 - abs(mir_orientation_to_litter)

        elif center < resolution_x/2:
            print('Smaller <')
            mir_aligned_orientation = mir_current_orientation + mir_orientation_to_litter
            if mir_current_orientation + mir_orientation_to_litter > 180:
                mir_aligned_orientation = abs(mir_orientation_to_litter) - 180

        # if vertical alignment does not hit bounding box then TRY AGAIN
        print('MiR Aligned Orientation: ',mir_aligned_orientation)

        # IMPLEMENTATION TIME

        if 0 <= float(mir_aligned_orientation) <= 90:
            x_sign = 1
            y_sign = 1

        elif 90 < float(mir_aligned_orientation) <= 180:
            x_sign = -1
            y_sign = 1

        elif 0 >= float(mir_aligned_orientation) >= -90:
            x_sign = 1
            y_sign = -1

        elif -90 > float(mir_aligned_orientation) >= -180:
            x_sign = -1
            y_sign = -1
        print('Sign_x: ', x_sign, 'Sign_y: ', y_sign)
        # prepare orientation for calculation


        if np.abs(mir_aligned_orientation) <= 90:
            mir_aligned_orientation = str(90 - abs(float(mir_aligned_orientation)))

        if np.abs(mir_aligned_orientation) > 90:
            if float(mir_aligned_orientation) < 0:
                mir_aligned_orientation = str(abs(float(mir_aligned_orientation) + 90))
            else:
                mir_aligned_orientation = str(abs(float(mir_aligned_orientation) - 90))

        # calculate coordinate for litter position
        litter_x = np.sin(float(mir_aligned_orientation) * (np.pi / 180)) * (cathetus_length / np.sin(np.pi / 2))
        litter_y = np.sqrt(cathetus_length ** 2 - litter_x ** 2)
        # change sign
        litter_x = (litter_x * x_sign) + float(mir.get_current_position()[0])
        litter_y = litter_y * y_sign + float(mir.get_current_position()[1])

        litter_list = (litter_x, litter_y, mir_aligned_orientation)
        print('litter_list: ', litter_list)

        return litter_list





'''
info= track.get_info()
depth_m = float(info[0])/1000
depth_m= float(cv.get_depth_information(depth_m))

center_X_point = int(info[1])
center_X_point = int(cv.get_litter_center_point(center_X_point))


cathetus_lenght = cv.cathethus_length(depth_m)
mir_aligned_orientation = float(cv.calculate_orientation_mir(depth_m,center_X_point,cathetus_lenght))

mir.litter_operation(cathetus_lenght,mir_aligned_orientation)
'''

