#!/usr/bin/env python3
# Configure depth and color streams
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
# Get the sensor once at the beginning. (Sensor index: 1)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.exposure, 600)

depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.exposure, 50000)
depth_scale = depth_sensor.get_depth_scale()

align_to = rs.stream.color
align = rs.align(align_to)

frames = pipeline.wait_for_frames()

aligned_frames = align.process(frames)
aligned_depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()

depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())



# Render images
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)

# Filename
path = 'C:/Users/aatefi2/Desktop/Intel real sense/Codes/'
imageName1 = str(time.strftime("%Y_%m_%d_%H_%M_%S")) +  '_Color.jpg'
imageName2 = str(time.strftime("%Y_%m_%d_%H_%M_%S")) +  '_Depth.jpg'

# Saving the image
cv2.imwrite(imageName1, color_image)
cv2.imwrite(imageName2, depth_image)
#cv2.imwrite(imageName3, images)
#cv2.imwrite(imageName4, bg_removed )
#cv2.imwrite(imageName5, depth_colormap )

key = cv2.waitKey(1)
# Press esc or 'q' to close the image window
cv2.destroyAllWindows()

pipeline.stop()


