#!/usr/bin/env python3
import cv2
import numpy as np
import numpy.ma
from math import sqrt
from depthIntel import *
import time


class frontCam():
    def __init__(self):
        self.dc = DepthCamera('141222072962')
        self.pt1 = (100, 420)
        self.pt2 = (600, 420)
        self.color = (0, 0, 255)
        self.kernel = np.array([[0, 0, 1, 0, 0], [0, 1, 1, 1, 0], [1, 1, 1, 1, 1], [0, 1, 1, 1, 0], [0, 0, 1, 0, 0]],
                               np.uint8)

    def front_cam(self):

        ret, depth_frame, color_frame = self.dc.get_frame()
        cropped_image = depth_frame[360:640, 30:450]

        a = cropped_image[60][:]

        #cv2.imshow('depth_frame',cropped_image)
        #cv2.line(color_frame, self.pt1, self.pt2, self.color)
        #cv2.imshow("Color frame", cropped_image)
        #cv2.waitKey(1)
        # time.sleep(0.003)
        average = min(a[np.nonzero(a)])

        mean = np.mean(a)
        print(average)
        if average < 1130:
            print('Average')
            #cv2.imshow('Object Detected', color_frame)
            #cv2.waitKey(1000)

            return True, color_frame, depth_frame

        else:

            return False, False, False

    def front_contour(self, color_frame, depth_frame):
        #bund = np.array([])
        #center = np.array([])
        bund = []
        center = []
        count = 0

        img_copy = color_frame
        color_frame = cv2.GaussianBlur(color_frame, (5, 5), 0)

        edges = cv2.Canny(color_frame, 0, 120, cv2.THRESH_BINARY)
        fix = cv2.dilate(edges, self.kernel, iterations=3)

        imf = fix.copy()

        h, w = imf.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(edges, mask, (50, 50), 255)

        e = cv2.erode(imf, self.kernel, iterations=3)

        black = np.zeros((e.shape[0], e.shape[1], 3), np.uint8)
        black1 = cv2.rectangle(black, (50, 350), (600, 465), (255, 255, 255), -1)  # ---the dimension of the ROI
        gray = cv2.cvtColor(black, cv2.COLOR_BGR2GRAY)  # ---converting to gray
        ret, b_mask = cv2.threshold(gray, 127, 255, 0)

        fin = cv2.bitwise_and(e, e, mask=b_mask)

        cv2.imshow('test', fin)
        cv2.waitKey(1)

        contours, hierarchy = cv2.findContours(image=fin, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            b = h + y
            c = x + w / 2
            #bund = np.append(bund, b)
            #center=np.append(center, c)
            bund.append(b)
            center.append(c)
            print('B: ', b)
            print('C: ', c)
            print('Bund: ', bund)
            print('Center: ', center)
            cv2.rectangle(color_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.drawContours(img_copy, contours, -1, (0, 255, 0), 3)

        def argmax(a):
            try:
                return max(range(len(a)), key=lambda x: a[x])
            except ValueError:
                return 0
        img_copy = img_copy
        cv2.drawContours(image=img_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2,
                         lineType=cv2.LINE_AA)
        #cv2.imshow('All Contours', img_copy)
        #cv2.waitKey(0)
        y, x = np.array([bund[argmax(bund)], center[argmax(bund)]])
        print('Y: ',y,' X: ', x)

        depth = depth_frame[round(y) - 1, round(x)]
        litter_list = [x, y, depth]
        return litter_list
'''
front = frontCam()
count = 0

while count < 20:
        ret, depth_frame, color_frame = front.dc.get_frame()
        count += 1
while True:
    
    result = front.front_cam()
    front.front_contour(result[1], result[2])
    if result[0] != 0:
        print('object found')
        print(front.front_contour(result[1], result[2]))
'''