#!/usr/bin/env python3
import requests, json
import time
import numpy as np

import cv2 as cv


class MiR():

    # HTTP Headers
    def __init__(self):

        # Endpoint
        self.state =0
        self.host = "http://mir.com/api/v2.0.0/"
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Accept-Language'] = 'en_US'
        self.headers[
            'Authorization'] = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='



    #RobotLab
    #Missions:
    #followPath: '0cfdf096-659b-11ed-8454-000129abea0a'
    #moveHome: '760f12e9-659b-11ed-8454-000129abea0a'
    #moveLitter: '40fe63da-659d-11ed-8454-000129abea0a'
    #litterEnd: '566345ae-659d-11ed-8454-000129abea0a'

    #Positions:
    #litter: '7f5cd607-4ade-11ed-a056-000129abea0a'
    #litterEnd: '7575aa03-4ade-11ed-a056-000129abea0a'

    #Map ID
    #RobotLab: 'd395fa10-449f-11ed-92c0-000129abea0a'

    # ---- # missions # ---- #
    # followPath: 'a251ecc4-4bb5-11ed-9e0f-000129abea0a'
    # sweepPath: '4e34c35d-4bb1-11ed-80ee-000129abea0a'
    # moveToLitter: '5da6da01-4bb9-11ed-9e0f-000129abea0a'
    # litterEnd: '6b8d1e03-4bb9-11ed-9e0f-000129abea0a'
    # afterLitterPickup1: '0778cfe6-4bba-11ed-9e0f-000129abea0a'
    # afterLitterPickup2: '2ee1f9b3-4bba-11ed-9e0f-000129abea0a'
    # afterLitterPickup3: '65deaeef-4bba-11ed-9e0f-000129abea0a'
    # move_to_home: '9181403d-4bc7-11ed-9e0f-000129abea0a'
    # false_positive: 'ebde55e5-4bc8-11ed-9e0f-000129abea0a'
    #mission_accomplidhed: 'b537b65d-591b-11ed-a937-000129abea0a'
    # mission_failed: 'c6554b3e-591b-11ed-a937-000129abea0a'

    # ---- # positions # ---- #
    # endMission: '9cd744e8-4ba7-11ed-825a-000129abea0a'
    # home: 'ba398174-4ba7-11ed-825a-000129abea0a'
    # waypoint1: '1911eb85-4ba9-11ed-825a-000129abea0a'
    # waypoint2: '1fb51d77-4ba9-11ed-825a-000129abea0a'
    # waypoint3: 'a4c3f98b-4ba9-11ed-825a-000129abea0a'
    # waypoint4: 'b58fc629-4ba9-11ed-825a-000129abea0a'
    # litter: 'e01afbdf-4bb3-11ed-9e0f-000129abea0a'
    # litterEnd: 'e49f9f23-4bb3-11ed-9e0f-000129abea0a'

    # ---- # maps # ---- #
    # motion capture lab id: '7ddad83a-4ba5-11ed-825a-000129abea0a'

    # get the given information
    def get_info(self):
        try:
            # system_info = requests.get(self.host + '/status', headers=self.headers)
            get_missions = requests.get(self.host + '/maps',
                                        headers=self.headers)  # get_maps = requests.get(self.host + 'maps', headers=self.headers)  # get_positions = requests.get(self.host + '/positions', headers=self.headers)
        except:
            return 'No connection'

        return get_missions.json()

    # get the system information
    def get_system_info(self):
        try:
            # Get is the HTTP Method
            system_info = requests.get(self.host + '/status', headers=self.headers)
        except:
            return 'No connection'

        return system_info.json()

    # check if mission queue is empty
    def get_mission_queue_status(self):
        sys_info = self.get_system_info()
        mission_queue_status = (sys_info['mission_queue_url'])

        return mission_queue_status

    # get the robot position
    def get_current_position(self):
        sys_info = self.get_system_info()
        current_position = (sys_info['position']['x'], sys_info['position']['y'], sys_info['position']['orientation'])

        return current_position

    # post follow_path
    '''
    def post_follow_path(self):
        follow_path_id = {'mission_id': 'a251ecc4-4bb5-11ed-9e0f-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=follow_path_id, headers=self.headers)

        return post_mission_queue
    '''
    def post_follow_path(self):
        follow_path_id = {'mission_id': '0cfdf096-659b-11ed-8454-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=follow_path_id, headers=self.headers)

        return post_mission_queue

    # post sweep_area 4e34c35d-4bb1-11ed-80ee-000129abea0a
    def post_sweep_area(self):
        sweep_area_id = {'mission_id': '4e34c35d-4bb1-11ed-80ee-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=sweep_area_id, headers=self.headers)

        return post_mission_queue

    # post move_to_home
    #def post_move_to_home(self):
        # body
        #move_to_home_id = {'mission_id': '9181403d-4bc7-11ed-9e0f-000129abea0a'}
        # post is HTTP Method
        #post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_home_id, headers=self.headers)

        #return post_mission_queue
    def post_move_to_home(self):
        # body
        move_to_home_id = {'mission_id': '760f12e9-659b-11ed-8454-000129abea0a'}
        # post is HTTP Method
        post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_home_id, headers=self.headers)

        return post_mission_queue

    # post move_to_litter
    #def post_move_to_litter(self):
        #move_to_litter_id = {'mission_id': '5da6da01-4bb9-11ed-9e0f-000129abea0a'}
        #post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_litter_id, headers=self.headers)

        #return post_mission_queue
    def post_move_to_litter(self):
        move_to_litter_id = {'mission_id': '40fe63da-659d-11ed-8454-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_litter_id, headers=self.headers)

        return post_mission_queue

    # post move_to_litter_end
    #def post_move_to_litter_end(self):
        #move_to_litter_end_id = {'mission_id': '6b8d1e03-4bb9-11ed-9e0f-000129abea0a'}
        #post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_litter_end_id,
                                           #headers=self.headers)

        #return post_mission_queue

    def post_move_to_litter_end(self):
        move_to_litter_end_id = {'mission_id': '566345ae-659d-11ed-8454-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=move_to_litter_end_id,
                                           headers=self.headers)

        return post_mission_queue

    # post false_positive
    def post_false_positive(self):
        false_positive_id = {'mission_id': 'ebde55e5-4bc8-11ed-9e0f-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=false_positive_id, headers=self.headers)

        return post_mission_queue

    # post after_litter_pickup1
    def post_after_litter_pickup1(self):
        after_litter_pickup1 = {'mission_id': '0778cfe6-4bba-11ed-9e0f-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=after_litter_pickup1,
                                           headers=self.headers)

        return post_mission_queue

    # post after_litter_pickup2
    def post_after_litter_pickup2(self):
        after_litter_pickup2 = {'mission_id': '2ee1f9b3-4bba-11ed-9e0f-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=after_litter_pickup2,
                                           headers=self.headers)

        return post_mission_queue

    # post after_litter_pickup3
    def post_after_litter_pickup3(self):
        after_litter_pickup3 = {'mission_id': '65deaeef-4bba-11ed-9e0f-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=after_litter_pickup3,
                                           headers=self.headers)

        return post_mission_queue

    def post_mission_accomplished(self):
        mission_accomplished = {'mission_id': 'b537b65d-591b-11ed-a937-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=mission_accomplished,
                                           headers=self.headers)

        return post_mission_queue

    def post_mission_failed(self):
        mission_failed = {'mission_id': 'c6554b3e-591b-11ed-a937-000129abea0a'}
        post_mission_queue = requests.post(self.host + '/mission_queue', json=mission_failed,
                                           headers=self.headers)

        return post_mission_queue

    # delete all missions
    def delete_all_missions(self):
        delete_mission_queue = requests.delete(self.host + '/mission_queue', headers=self.headers)

        return delete_mission_queue

    # check mission queue
    def get_mission_queue(self):
        mission_queue = requests.get(self.host + '/mission_queue', headers=self.headers)

        return mission_queue

    # change values for litter
    #def put_litter_position(self, litter_coord):
    #    litter_position = {'name': 'litter', 'pos_x': litter_coord[0], 'pos_y': litter_coord[1],
    #                       'orientation': litter_coord[2], 'type_id': 0,
    #                       'map_id': '7ddad83a-4ba5-11ed-825a-000129abea0a'}
    #
    #    put_litter_position = requests.put(self.host + 'positions/e01afbdf-4bb3-11ed-9e0f-000129abea0a',
    #                                       json=litter_position, headers=self.headers)
    #
    #    return put_litter_position

    def put_litter_position(self, litter_coord):
        litter_position = {'name': 'litter', 'pos_x': litter_coord[0], 'pos_y': litter_coord[1],
                           'orientation': litter_coord[2], 'type_id': 0,
                           'map_id': 'd395fa10-449f-11ed-92c0-000129abea0a'}

        put_litter_position = requests.put(self.host + 'positions/7f5cd607-4ade-11ed-a056-000129abea0a',
                                           json=litter_position, headers=self.headers)

        return put_litter_position

    # change values for litter_end
    def put_litter_end_position(self):
        litter_end_position = {'name': 'litterEnd', 'pos_x': self.get_current_position()[0],
                               'pos_y': self.get_current_position()[1], 'orientation': self.get_current_position()[2],
                               'type_id': 0, 'map_id': 'd395fa10-449f-11ed-92c0-000129abea0a'}

        put_litter_end_position = requests.put(self.host + 'positions/7575aa03-4ade-11ed-a056-000129abea0a',
                                               json=litter_end_position, headers=self.headers)

        return put_litter_end_position

    # change values for chosen position
    def put_any_position(self, guid, name, pos):
        position = {'name': name, 'pos_x': pos[0], 'pos_y': pos[1], 'orientation': pos[2], 'type_id': 0,
                    'map_id': '7ddad83a-4ba5-11ed-825a-000129abea0a'}

        put_litter_position = requests.put(self.host + 'positions/' + guid, json=position, headers=self.headers)

        return put_litter_position

    # navigation for followPath
    def follow_path(self):

        pos1 = (4.550, 2.750, 0)  # x, y, ori
        pos2 = (6.850, 2.750, 90)
        pos3 = (7.1, 5.5, 90)
        print(self.state)

        if self.state is 0:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos1)
            self.state+=1
        if self.get_mission_queue_status() is  None and self.state is 1:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos1)
            self.post_follow_path()
            self.state += 1
            time.sleep(1)
        elif self.get_mission_queue_status() is  None and self.state is 2:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos2)
            self.post_follow_path()
            self.state+=1
            time.sleep(1)
        elif self.get_mission_queue_status() is  None and self.state is 3:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos3)
            self.post_follow_path()
            self.state += 1
        return 0

    def sweep_area(self):

        pos1 = (4.550, 2.750, 0)  # x, y, ori
        pos2 = (5.850, 2.650, 90)
        pos3 = (5.850, 5.450, 0)
        pos4 = (7.1, 5.4, -90)
        pos5 = (7.1, 2.7, -90)

        print(self.state)

        if self.state is 0:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos1)
            self.state+=1
        if self.get_mission_queue_status() is None and self.state is 1:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos1)
            self.post_follow_path()
            self.state += 1
            time.sleep(1)
        elif self.get_mission_queue_status() is None and self.state is 2:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos2)
            self.post_follow_path()
            self.state+=1
            time.sleep(1)
        elif self.get_mission_queue_status() is None and self.state is 3:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos3)
            self.post_follow_path()
            self.state += 1
            time.sleep(1)
        elif self.get_mission_queue_status() is None and self.state is 4:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos4)
            self.post_follow_path()
            self.state += 1
            time.sleep(1)
        elif self.get_mission_queue_status() is None and self.state is 5:
            self.put_any_position('1911eb85-4ba9-11ed-825a-000129abea0a', 'mission_pos', pos5)
            self.post_follow_path()
            self.state += 1
        return 0
    # litter operation
    def litter_operation(self, cathetus_length, mir_aligned_orientation, depth):
        # vertically align, in horizontal center with tracking

        # calculate distance to object in meters

        # get distance of current view
        # use classification on center depth pixels of object to find hyp
        hypotenuse = depth  # depth measured in meters
        cathetus_height = 0.35  # some measured constant in meters
        current_position = self.get_current_position()

        # check for ++, +-, -+, --
        if 0 <= float(current_position[2]) <= 90:
            x_sign = 1
            y_sign = 1

        elif 90 < float(current_position[2]) <= 180:
            x_sign = -1
            y_sign = 1

        elif 0 >= float(current_position[2]) >= -90:
            x_sign = 1
            y_sign = -1

        elif -90 > float(current_position[2]) >= -180:
            x_sign = -1
            y_sign = -1
        # prepare orientation for calculation
        current_position_list = list(current_position)

        if np.abs(current_position[2]) <= 90:
            current_position_list[2] = str(90 - abs(float(current_position[2])))

        if np.abs(current_position[2]) > 90:
            if float(current_position[2]) < 0:
                current_position_list[2] = str(abs(float(current_position[2]) + 90))
            else:
                current_position_list[2] = str(abs(float(current_position[2]) - 90))

        # calculate coordinate for litter position
        litter_x = np.sin(float(current_position_list[2]) * (np.pi / 180)) * (cathetus_length / np.sin(np.pi / 2))
        litter_y = np.sqrt(cathetus_length ** 2 - litter_x ** 2)
        # change sign
        litter_x = (litter_x * x_sign) + float(current_position[0])
        litter_y = litter_y * y_sign + float(current_position[1])

        litter_list = (litter_x, litter_y, mir_aligned_orientation)

        self.put_litter_position(litter_list)



# run commands
'''
# test scenario
mir.post_move_to_home()
mir.post_follow_path()
time.sleep(5)
print('found some litter my dudes!')
mir.delete_all_missions()
mir.put_litter_end_position()
mir.put_litter_position()
mir.post_move_to_litter()
print('picking up litter with insane computer vision strats')
mir.post_after_litter_pickup1()
# mir.post_false_positive()
mir.post_move_to_litter_end()
mir.post_follow_path()
mir.post_move_to_home()
'''
'''
# first nav test
mir.post_move_to_home()
while mir.get_mission_queue_status() is not None:
    print(mir.get_current_position())
mir.init_follow_path()
'''  # print(mir.litter_operation())
'''
mir.litter_operation()
mir.post_move_to_litter()
'''