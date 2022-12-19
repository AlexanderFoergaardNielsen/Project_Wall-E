#!/usr/bin/env python3
import cv2
import numpy as np
import imutils
import os
from end_effector_data import *
from depthIntel import *

from scipy.spatial import distance as dist


class URAlignment():
    def __init__(self):
        self.dc = DepthCamera('827312070023')
        self.res_x = 640
        self.res_y = 420

    def test(self):
        while True:
            ret, depth_frame, color_frame = self.dc.get_frame()
            cv2.imshow('test', color_frame)
            cv2.waitKey(1)

    def get_all_real_time(self):
        count = 0

        while count < 20:
            ret, depth_frame, color_frame = self.dc.get_frame()
            count += 1
        while True:
            ret, depth_frame, color_frame = self.dc.get_frame()
            cropped_color = color_frame[0:self.res_y, 0:self.res_x]
            cropped_depth = depth_frame[0:self.res_y, 0:self.res_x]

            # cv2.imwrite('color.png', cropped_color)

           # np.save('empty_depth.npy', cropped_depth)
           # cv2.waitKey(0)
            template = np.load('/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/IntelPythonOpenCV/empty_depth.npy')

            #template = np.load(
            #    '/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/Movement_folder/IntelPythonOpenCV/empty_depth.npy')

            diff = cv2.absdiff(template, cropped_depth)

            diff[diff < 25] = 0
            diff[diff > 25] = 255

            diff = cv2.medianBlur(diff, 5)

            kernel = np.ones((5, 5), np.uint8)

            diff = cv2.morphologyEx(diff, cv2.MORPH_OPEN, kernel)

            mask = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX) / 255

            mask = (mask * 255).round().astype(np.uint8)

            diff = cv2.bitwise_not(mask)

            backToRBG = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)

            black = np.where(backToRBG == 0)

            backToRBG[black[0], black[1], :] = [0, 0, 255]

            apply_mask = cv2.bitwise_and(cropped_color, backToRBG)

            cv2.waitKey(1)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            # gamer crosshair
            line_thickness = 3
            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 - 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 - 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 + 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 + 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 - 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 - 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 + 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 + 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            hyps = []

            for c in cnts:
                if cv2.contourArea(c) > 250:
                    # compute the center of the contour
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # draw the contour and center of the shape on the image
                    cv2.drawContours(cropped_color, [c], -1, (0, 255, 0), 2)
                    cv2.circle(cropped_color, (cX, cY), 7, (0, 0, 0), -1)
                    cv2.putText(cropped_color, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255), 2)

                    rotrect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rotrect)
                    box = np.int0(box)

                    # draw rotated rectangle on copy of img as result
                    cv2.drawContours(cropped_color, [box], 0, (0, 0, 255), 2)

                    # get angle from rotated rectangle
                    angle = rotrect[-1]

                    # from https://www.pyimagesearch.com/2017/02/20/text-skew-correction-opencv-python/
                    # the `cv2.minAreaRect` function returns values in the
                    # range [-90, 0); as the rectangle rotates clockwise the
                    # returned angle trends to 0 -- in this special case we
                    # need to add 90 degrees to the angle
                    if angle < -45:
                        angle = -(90 + angle)

                    # otherwise, just take the inverse of the angle to make
                    # it positive
                    else:
                        angle = -angle

                    # print(angle, "deg")

                    x_length = 715
                    y_length = 465
                    x_resolution = self.res_x
                    y_resolution = self.res_y

                    x_dist = x_length * (cX - (x_resolution / 2)) / x_resolution / 1000.00
                    y_dist = y_length * (cY - (y_resolution / 2)) / y_resolution / 1000.00

                    hyp = np.sqrt((x_dist ** 2 + y_dist ** 2))
                    hyps.append((hyp, x_dist, y_dist, angle, c))

                    height = cropped_depth[cY][cX] / 1000.00

            hyps = np.array(hyps)
            cv2.imshow("Image", cropped_color)
            cv2.waitKey(1)

            if hyps.any():
                hyps_sort = np.sort(hyps[:, 0])
                closest = hyps[np.where(hyps[:, 0] == hyps_sort[0])]
                closest_list = [closest[0, 1], closest[0, 2], closest[0, 3], height]
                print(closest_list)

    def get_object_position(self):
        count = 0

        while count < 20:
            ret, depth_frame, color_frame = self.dc.get_frame()
            count += 1

        while True:
            ret, depth_frame, color_frame = self.dc.get_frame()
            cropped_color = color_frame[0:self.res_y, 0:self.res_x]
            cropped_depth = depth_frame[0:self.res_y, 0:self.res_x]

            # cv2.imwrite('color.png', cropped_color)

            # np.save('empty_depth.npy', cropped_depth)

            template = np.load('/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/IntelPythonOpenCV/empty_depth.npy')

            #template = np.load(
            #    '/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/Movement_folder/IntelPythonOpenCV/empty_depth.npy')

            diff = cv2.absdiff(template, cropped_depth)

            diff[diff < 25] = 0
            diff[diff > 25] = 255

            diff = cv2.medianBlur(diff, 5)

            kernel = np.ones((5, 5), np.uint8)

            diff = cv2.morphologyEx(diff, cv2.MORPH_OPEN, kernel)

            mask = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX) / 255

            mask = (mask * 255).round().astype(np.uint8)

            diff = cv2.bitwise_not(mask)

            backToRBG = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)

            black = np.where(backToRBG == 0)

            backToRBG[black[0], black[1], :] = [0, 0, 255]

            apply_mask = cv2.bitwise_and(cropped_color, backToRBG)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            # gamer crosshair
            line_thickness = 3
            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 - 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 - 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 + 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 + 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 - 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 - 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 + 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 + 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            hyps = []

            for c in cnts:
                if cv2.contourArea(c) > 250:
                    # compute the center of the contour
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # draw the contour and center of the shape on the image
                    cv2.drawContours(cropped_color, [c], -1, (0, 255, 0), 2)
                    cv2.circle(cropped_color, (cX, cY), 7, (255, 255, 255), -1)
                    cv2.putText(cropped_color, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255), 2)

                    x_length = 715
                    y_length = 465
                    x_resolution = self.res_x
                    y_resolution = self.res_y

                    x_dist = float(x_length * (cX - (x_resolution / 2)) / x_resolution / 1000.00)
                    y_dist = float(y_length * (cY - (y_resolution / 2)) / y_resolution / 1000.00)

                    hyp = np.sqrt((x_dist ** 2 + y_dist ** 2))
                    hyps.append((hyp, x_dist, y_dist))

            hyps = np.array(hyps)

            cv2.imshow("Image", cropped_color)
            cv2.waitKey(1)

            if hyps.any():
                hyps_sort = np.sort(hyps[:, 0])
                closest = hyps[np.where(hyps[:, 0] == hyps_sort[0])]
                closest_list = [closest[0, 1], closest[0, 2]]
                print(closest_list)

                return closest_list

            if not hyps.any():
                return -1

    def get_angle_depth_object(self):
        count = 0

        while count < 20:
            ret, depth_frame, color_frame = self.dc.get_frame()
            count += 1
        while True:
            ret, depth_frame, color_frame = self.dc.get_frame()
            cropped_color = color_frame[0:self.res_y, 0:self.res_x]
            cropped_depth = depth_frame[0:self.res_y, 0:self.res_x]

            # cv2.imwrite('color.png', cropped_color)

            # np.save('empty_depth.npy', cropped_depth)

            template = np.load('/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/IntelPythonOpenCV/empty_depth.npy')

            #template = np.load(
            #    '/home/alexander/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/Movement_folder/IntelPythonOpenCV/empty_depth.npy')

            diff = cv2.absdiff(template, cropped_depth)

            diff[diff < 25] = 0
            diff[diff > 25] = 255

            diff = cv2.medianBlur(diff, 5)

            kernel = np.ones((5, 5), np.uint8)

            diff = cv2.morphologyEx(diff, cv2.MORPH_OPEN, kernel)

            mask = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX) / 255

            mask = (mask * 255).round().astype(np.uint8)

            diff = cv2.bitwise_not(mask)

            backToRBG = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)

            black = np.where(backToRBG == 0)

            backToRBG[black[0], black[1], :] = [0, 0, 255]

            apply_mask = cv2.bitwise_and(cropped_color, backToRBG)

            cv2.waitKey(1)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            lst_intensities = []

            # For each list of contour points...

            # gamer crosshair
            line_thickness = 3
            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 - 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 - 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2), int(self.res_y / 2 + 12)),
                     (int(self.res_x / 2), int(self.res_y / 2 + 5)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 - 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 - 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            cv2.line(cropped_color, (int(self.res_x / 2 + 5), int(self.res_y / 2)),
                     (int(self.res_x / 2 + 12), int(self.res_y / 2)), (0, 0, 255), thickness=line_thickness)

            result = []

            object = 1 # 1 = waste, 2 = paper, 3 = plastic

            for c in cnts:
                if cv2.contourArea(c) > 250:
                    # compute the center of the contour
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # draw the contour and center of the shape on the image

                    rotrect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rotrect)
                    box = np.int0(box)

                    # draw rotated rectangle on copy of img as result
                    cv2.drawContours(cropped_color, [box], 0, (0, 0, 255), 2)

                    angle = rotrect[-1]
                    # angle = abs(angle)
                    print('Angle before transform: ', angle)
                    angle = abs(angle)
                    print('Absolute Angle: ', angle)
                    dist_0_3 = dist.euclidean((box[0, 0], box[0, 1]), (box[1, 0], box[1, 1]))
                    dist_0_1 = dist.euclidean((box[0, 0], box[0, 1]), (box[3, 0], box[3, 1]))
                    print('Angle Side: ', dist_0_3)
                    print('Other Side: ', dist_0_1)

                    if dist_0_3 > dist_0_1:
                        print('Long Side: ')
                        angle = - (angle)

                    elif dist_0_3 < dist_0_1:
                        angle = 90 - angle

                    print('Sent Angle: ', angle)


                    x_length = 715
                    y_length = 465
                    x_resolution = self.res_x
                    y_resolution = self.res_y

                    x_dist = x_length * (cX - (x_resolution / 2)) / x_resolution / 1000.00
                    y_dist = y_length * (cY - (y_resolution / 2)) / y_resolution / 1000.00
                    height = cropped_depth[cY][cX]/1000.00
                    '''
                    for i in range(len(cnts)):
                        # Create a mask image that contains the contour filled in
                        cimg = np.zeros_like(cropped_color)
                        cv2.drawContours(cimg, cnts, i, color=255, thickness=-1)

                        # Access the image pixels and create a 1D numpy array then add to list
                        pts = np.where(cimg == 255)
                        x = pts[0]
                        y = pts[1]
                       # print('Cropped depth: ', np.shape(cropped_depth))
                       # print('X Y',x,y)

                
                        for p in range(len(pts[1])-1):

                            yPoint = y[p]
                            #print('Y point: ',yPoint)
                            xPoint = x[p]
                           # print('X point: ', xPoint)
                            depth_value = cropped_depth[xPoint][yPoint]
                            lst_intensities.append(depth_value)
                            #depth_value =cropped_depth[x][y]
                           # print('Depth value: ' + str(depth_value))
                            lst_intensities.append(depth_value)
            
                    lst_intensities = np.sort(lst_intensities, axis=None)
                    height = np.median(lst_intensities) / 1000
                '''

                    hyp = np.sqrt((x_dist ** 2 + y_dist ** 2))
                    result.append((hyp, x_dist, y_dist, angle, height, object, c))

            result = np.array(result)
            #print(result[0,6])
            cv2.imshow("Image", cropped_color)
            cv2.waitKey(1)

            if result.any():
                hyps_sort = np.sort(result[:, 0])
                result = result[np.where(result[:, 0] == hyps_sort[0])]
                c = result[0, 6]
                rotrect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rotrect)
                box = np.int0(box)

                w = np.sqrt(abs(box[0][0] - box[1][0]) ** 2 + abs(box[0][1] - box[1][1]) ** 2)
                h = np.sqrt(abs(box[0][0] - box[3][0]) ** 2 + abs(box[0][1] - box[3][1]) ** 2)
                perimeter = cv2.arcLength(c, True)
                area = cv2.contourArea(c)
                #comp = area / (w * h)

                circ = (4 * np.pi * area) / (perimeter ** 2)
                test = np.array([circ, area])
                #print([circ, comp, 2], ",")
                nabo = get_neighbors(ap, test, 9)
                print(nabo)
                prediction = predict_classification(ap, test, 9)

                prediction = int(prediction)
                result_list = [result[0, 3], result[0, 4], prediction]
                print(result_list)

                return result_list

            if not result.any():
                return 0
#align = URAlignment()
#align.get_all_real_time()
#while True:
#    align.get_object_position()
#align.get_object_position()
#align.get_angle_depth_object()
# align.test()
