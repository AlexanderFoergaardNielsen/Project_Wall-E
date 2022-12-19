#!/usr/bin/env python3

import time
from mir_navigation import MiR
#from Tracking import Track
from litter_alignment import ComputerVision
from draw_line import frontCam
import URMove

cv = ComputerVision()
mir = MiR()
front = frontCam()
import PySimpleGUI as sg
#GUI setup
relevantpath = False

sg.theme('DarkBlack')

selectMission = (
    'Follow path', 'Sweep area')

width = max(map(len, selectMission))+1

font = ("Arial", 18)


layout = [
    [sg.Button('Return home', key='RH', font=font, visible=True)],
    [sg.Button('Run', key='SI', font=font, visible=True),
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
        #print('Initialising Programme...')
        #launch init script
        #print('Initialisation done!')
        choose_mission = "F"
        break
print(choose_mission)
window.close()
time.sleep(0.5)
litterDetected=False
if choose_mission == 'F':
    mir.post_follow_path()
    print("F has been choosen")

    count = 0

    while count < 20:
        ret, depth_frame, color_frame = front.dc.get_frame()
        count += 1
        print("this is itterating")

    print(mir.get_mission_queue_status())

    while mir.get_mission_queue_status() is not None:  # While mission_que status =! None

        print("mission qeue is differenct from 0")

        result = front.front_cam()

        if result[0] != 0:
            print('object found')
            print(front.front_contour(result[1], result[2]))
            litterDetected = True

        else:
            print('-1')
        if litterDetected == True:
            mir.delete_all_missions()

            # save current position to get back after litter operation
            mir.put_litter_end_position()

            # align robot to the litter
            center_X_point, center_Y_point, depth_m =front.front_contour(result[1], result[2])
            depth_m = front.front_contour(result[1], result[2])[2]/1000.00
            depth_m = float(depth_m)
            depth_m = float(cv.get_depth_information(depth_m))
            print('Depth: ',depth_m,' X :', center_X_point)
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

            center_X_point, center_Y_point, depth_m = front.front_contour(result[1], result[2])
            depth_m = float(front.front_contour(result[1], result[2])[2]) / 1000.00
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
    mir.sweep_area()
    mir.post_sweep_area()

    while mir.state < 6:  # While mission_que status != None

        mir.sweep_area()

        # if keyboard.is_pressed('l'):  # if key 'q' == pressed
        # litterDetected = True
        '''
        if litterDetected == True:
            mir.delete_all_missions()

            # save current position to get back after litter operation
            mir.put_litter_end_position()

            # align robot to the litter
            info = track.get_info()
            depth_m = float(info[0]) / 1000
            depth_m = float(cv.get_depth_information(depth_m))
            center_X_point = int(info[1])
            center_X_point = int(cv.get_litter_center_point(center_X_point))

            cathetus_lenght = cv.cathethus_length(depth_m)

            mir_aligned = cv.calculate_orientation_mir(depth_m, center_X_point, cathetus_lenght,
                                                       mir.get_current_position()[2])
            pos_mir = (mir.get_current_position()[0], mir.get_current_position()[1], mir_aligned)
            mir.put_litter_position(pos_mir)
            time.sleep(1)
            mir.post_move_to_litter()
            time.sleep(5)
            while mir.get_mission_queue_status() != None:
                print('Waiting for alignment:')
                time.sleep(1)
            info = track.get_info()
            depth_m = float(info[0]) / 1000
            depth_m = float(cv.get_depth_information(depth_m))
            center_X_point = int(info[1])
            center_X_point = int(cv.get_litter_center_point(center_X_point))
            cathetus_lenght = cv.cathethus_length(depth_m)
            mir.litter_operation(cathetus_lenght, mir_aligned, depth_m)

            mir.post_move_to_litter()

            litterDetected = False

            # Move UR5 arm to pickup position
            # Collect litter and go back to pickup position
            # Put litter in right bin
            # Go back to IDLE position

            #
            mir.post_move_to_litter_end()
            if choose_mission == 'F':
                mir.post_follow_path()
            elif choose_mission == 'S':
                mir.post_sweep_area()
        '''
elif choose_mission == 'T':
    print('Test Mission.')
else:
    print('Invalid input.')
    exit()
litterDetected = False
'''
while mir.state<4: # While mission_que status != None

    mir.follow_path()

    #if keyboard.is_pressed('l'):  # if key 'q' == pressed
        #litterDetected = True

    if litterDetected == True:
        mir.delete_all_missions()

        # save current position to get back after litter operation
        mir.put_litter_end_position()

        # align robot to the litter
        info = track.get_info()
        depth_m = float(info[0]) / 1000
        depth_m = float(cv.get_depth_information(depth_m))
        center_X_point = int(info[1])
        center_X_point = int(cv.get_litter_center_point(center_X_point))

        cathetus_lenght = cv.cathethus_length(depth_m)

        mir_aligned = cv.calculate_orientation_mir(depth_m, center_X_point, cathetus_lenght,
                                                   mir.get_current_position()[2])
        pos_mir = (mir.get_current_position()[0], mir.get_current_position()[1], mir_aligned)
        mir.put_litter_position(pos_mir)
        time.sleep(1)
        mir.post_move_to_litter()
        time.sleep(5)
        while mir.get_mission_queue_status() != None:
            print('Waiting for alignment:')
            time.sleep(1)
        info = track.get_info()
        depth_m = float(info[0]) / 1000
        depth_m = float(cv.get_depth_information(depth_m))
        center_X_point = int(info[1])
        center_X_point = int(cv.get_litter_center_point(center_X_point))
        cathetus_lenght = cv.cathethus_length(depth_m)
        mir.litter_operation(cathetus_lenght, mir_aligned, depth_m)

        mir.post_move_to_litter()

        litterDetected = False

        # Move UR5 arm to pickup position
        # Collect litter and go back to pickup position
        # Put litter in right bin
        # Go back to IDLE position

        #
        mir.post_move_to_litter_end()
        if choose_mission == 'F':
            mir.post_follow_path()
        elif choose_mission == 'S':
            mir.post_sweep_area()
'''
# detect litter in environment
# litter detected = True
#mir.post_mission_accomplished()
mir.post_move_to_home()
