#!/usr/bin/env python3
import pyrealsense2 as rs
ctx = rs.context()
serials = []
for i in range(len(ctx.devices)):
    sn = ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(sn)
    serials.append(sn)
#141222072962 - FRONTCAMERA
#827312070023 - URCAMERA
config = rs.config()
config.enable_device(serials[0])

pipeline = rs.pipeline()
pipeline.start(config)