#!/usr/bin/env python3

import time
from mir_navigation import MiR
#from Tracking import Track
from litter_alignment import ComputerVision
#from draw_line import frontCam
from YoloScript import Yolo
import URMove

cv = ComputerVision()
mir = MiR()
#front = frontCam()
yolo = Yolo()
#track = Track()

import PySimpleGUI as sg
#GUI setup
relevantpath = False

sg.theme('DarkBlack')

selectMission = (
    'Follow path', 'Sweep area')

width = max(map(len, selectMission))+1

font = ("Arial", 18)


layout = [
    [sg.Text("Select mission", font=font)], [sg.Combo(selectMission, size=(width, 5), font=font, enable_events=True, key='SBCC')],
    [sg.Button('Return home', key='RH', font=font, visible=True)],
    [sg.Button('Confirm', key='SI', font=font, visible=True),
     sg.Cancel('Exit', font=font, key='EXIT')]
]

window = sg.Window("Litter Robot Controller", layout, margins=(50, 20), finalize=True)

#Main loop for GUI

while True:
    event, values = window.read()
    #print(event, values)

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'EXIT':
        break

    if event == 'RH':
        print('Returning home...')
        mir.post_move_to_home()

    if event == 'SI':
        if values['SBCC'] == "":
            sg.popup_error(f'Choose a mission please!', font=font)
        else:
            print('Initialising Programme...')
            #launch init script
            print('Initialisation done!')
            if values['SBCC'] == "Follow path":
                choose_mission = "F"
            elif values['SBCC'] == "Sweep area":
                choose_mission = "S"
            break
print(choose_mission)
window.close()

time.sleep(0.5)
litterDetected=False
if choose_mission == 'F':
    mir.post_follow_path()
    time.sleep(1)
    print('Follow path')
    count = 0


    #while count < 20:
    #    ret, depth_frame, color_frame = front.dc.get_frame()
    #    count += 1

    while mir.get_mission_queue_status() is not None:  # While mission_que status =! None



        #result = front.front_cam()
        '''
        if result[0] != 0:
            print('object found')
            print(front.front_contour(result[1], result[2]))
            litterDetected = True

        else:
            print('-1')
        '''
        alig = False
        litter_info = yolo.yolov5(alig)
        if sum(litter_info) > 0:
            print('Litter Detected')
            litterDetected = True
        else:
            print('Litter not detected')
        if litterDetected == True:

            mir.delete_all_missions()

            # save current position to get back after litter operation
            mir.put_litter_end_position()

            # align robot to the litter
            #center_X_point, center_Y_point, depth_m =front.front_contour(result[1], result[2])
            (center_X_point, center_Y_point, depth_m) = litter_info
            #depth_m = front.front_contour(result[1], result[2])[2]/1000.00
            depth_m = depth_m/1000.00
            depth_m = float(depth_m)
            depth_m = float(cv.get_depth_information(depth_m))
            print('Depth: ',depth_m,' X: ', center_X_point, ' Y: ', center_Y_point)
            center_X_point = int(cv.get_litter_center_point(center_X_point))
            ''' Check after if X is actually X but no'''

            cathetus_lenght = cv.cathethus_length(depth_m)

            mir_aligned = cv.calculate_orientation_mir(depth_m, center_X_point, cathetus_lenght,
                                                       mir.get_current_position()[2])

            x, y, ori = float(mir.get_current_position()[0]), float(mir.get_current_position()[1]), float(mir_aligned)
            pos_mir = (x,y,ori)
            print(mir.get_current_position()[2])
            print(pos_mir)
            mir.put_litter_position(pos_mir)
            time.sleep(1)
            mir.post_move_to_litter()
            time.sleep(1)
            while mir.get_mission_queue_status() != None:
                print('Waiting for alignment:')
                time.sleep(1)
            alig=True
            get_info_litter = yolo.yolov5(alig)
            attempsDetect=0
            while sum(get_info_litter) == 0:
                print('Attempts: ', attempsDetect)
                if attempsDetect>10:
                    alig=False

                    break
                get_info_litter = yolo.yolov5(alig)
                attempsDetect+=1

            if sum(get_info_litter) > 0:
                (center_X_point, center_Y_point, depth_m) = get_info_litter

            # depth_m = front.front_contour(result[1], result[2])[2]/1000.00
                depth_m = depth_m / 1000.00
                depth_m = float(depth_m)
                depth_m = float(cv.get_depth_information(depth_m))
                depth_m = float(cv.get_depth_information(depth_m))
                center_X_point = int(cv.get_litter_center_point(center_X_point))
                cathetus_lenght = cv.cathethus_length(depth_m)
                mir.litter_operation(cathetus_lenght, mir_aligned, depth_m)

                mir.post_move_to_litter()
                time.sleep(0.5)

                while mir.get_mission_queue_status() != None:
                    print('Robot is reaching for litter')
                    time.sleep(1)

                URMove.URmain()

            litterDetected = False
            alig=False

            # Move UR5 arm to pickup position
            # Collect litter and go back to pickup position
            # Put litter in right bin
            # Go back to IDLE position

            #

            mir.post_move_to_litter_end()
            time.sleep(1)
            while mir.get_mission_queue_status() != None:
                print('Waiting to return...')
            if choose_mission == 'F':
                mir.post_follow_path()
            elif choose_mission == 'S':
                mir.post_sweep_area()
            time.sleep(1)



elif choose_mission == 'S':
    # remember to make init sweep area
    print('Sweep Area')

else:
    print('Invalid input.')
    exit()
litterDetected = False

mir.post_move_to_home()
