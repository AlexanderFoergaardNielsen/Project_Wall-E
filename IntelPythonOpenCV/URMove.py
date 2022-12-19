#!/usr/bin/env python3

'''
geometry_msgs.Pose(
                geometry_msgs.Vector3(-0.20156, 0.21597, 0.24886),
                geometry_msgs.Quaternion(0.5958964, 0.8023778, -0.0324222, -0.0067906)) #Safe return to idle from waypoint

geometry_msgs.Pose(
                geometry_msgs.Vector3(0, -0.29408, 0.88807),
                geometry_msgs.Quaternion(0.3261029, -0.7989699, 0.4669508, 0.193031)) #bin approach









'''

import numpy as np
import sys
import time
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

#from YoloAlignment import YoloAlignment
from UR_alignment import URAlignment


# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class GripperControl:

    def publisher(self, stroke):
        print("publisher start")
        "Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic"
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

        self.command = outputMsg.Robotiq2FGripper_robot_output();
        self.command.rACT=1
        self.command.rGTO=1
        self.command.rSP=255
        self.command.rFR = 96
        self.command.rPR = stroke
        self.pub.publish(self.command)
        time.sleep(0.25)


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):

        self.gripperquarternion = [0.9238536, 0.3826808, -0.0063537, -0.0030927]
        self.gripperVector3 = [0.06240, 0.48551, 0.30074]
        self.iteration = 0
        self.latest_pos = [0.0, 0.0, 0.0]
        self.my_list = list(self.latest_pos)

        self.GraspPoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.06240, 0.48551, -0.20800),
            geometry_msgs.Quaternion(self.gripperquarternion[0], self.gripperquarternion[1], self.gripperquarternion[2], self.gripperquarternion[3]))  #Grasp near floor

        self.pickupPoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(self.gripperVector3[0], self.gripperVector3[1], self.gripperVector3[2]),
            geometry_msgs.Quaternion(self.gripperquarternion[0], self.gripperquarternion[1], self.gripperquarternion[2], self.gripperquarternion[3])) # Ready to grasp position

        self.pickupPoint_low = geometry_msgs.Pose(
            geometry_msgs.Vector3(self.gripperVector3[0], self.gripperVector3[1], self.gripperVector3[2]-0.1500),
            geometry_msgs.Quaternion(self.gripperquarternion[0], self.gripperquarternion[1], self.gripperquarternion[2],
                                     self.gripperquarternion[3]))  # Ready to grasp position

        self.idlePoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.10370, 0.211561, 0.25886),
            geometry_msgs.Quaternion(self.gripperquarternion[0], self.gripperquarternion[1], self.gripperquarternion[2], self.gripperquarternion[3]))  # neutral position

        self.wayPointReturn = geometry_msgs.Pose(
            geometry_msgs.Vector3(-0.33271, -0.01000, 0.55891),
            geometry_msgs.Quaternion(0.3674467, 0.9174423, -0.0676547, -0.1367673))  # safe waypoint

        self.wayPoint = geometry_msgs.Pose(
            #geometry_msgs.Vector3(-0.29000, 0.10000, 0.86365),
            geometry_msgs.Vector3(-0.30000, 0.0214, 0.84365),
            geometry_msgs.Quaternion(0.2659602, 0.6646579, -0.2600362, -0.6479786))  # safe waypoint

        self.binPreSafeApproachPoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(-0.18000, -0.30224, 0.88550),
            geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))  # bin approach

        self.binDropPoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.18000, -0.37870, 0.78191),
            geometry_msgs.Quaternion(0.3320563, -0.8176629, 0.4353422, 0.1778851))  # bin approach

        #self.binApproachPoint = geometry_msgs.Pose(
            #geometry_msgs.Vector3(0, -0.30224, 0.88550),
            #geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))  # bin approach

        self.bin1 = geometry_msgs.Pose(
            geometry_msgs.Vector3(-0.18000, -0.30224, 0.88550),
            geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))  # bin approach

        self.safeIdlePoint = geometry_msgs.Pose(
            geometry_msgs.Vector3(-0.20156, 0.21597, 0.24886),
            geometry_msgs.Quaternion(0.5958964, 0.8023778, -0.0324222, -0.0067906)) # Safe return to idle from waypoint




        rospy.init_node("URNode")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    def quaternion_converter(self, quaternion):
        quaternion[2], quaternion[3] = quaternion[3], quaternion[2]
        quaternion[1], quaternion[2] = quaternion[2], quaternion[1]
        quaternion[0], quaternion[1] = quaternion[1], quaternion[0]

        return quaternion

    def quaternion_to_ur(self, quaternion):
        quaternion[0], quaternion[1] = quaternion[1], quaternion[0]
        quaternion[1], quaternion[2] = quaternion[2], quaternion[1]
        quaternion[2], quaternion[3] = quaternion[3], quaternion[2]

        return quaternion

    def angle_to_quaternions(self, angle):
        # atitude = z
        angle = np.deg2rad(angle)
        c1 = 1
        c2 = np.cos(angle / 2)
        c3 = 1
        s1 = 0
        s2 = np.sin(angle / 2)
        s3 = 0

        quaternion = [0, 0, 0, 0]
        quaternion[3] = c1 * s2 * c3 - s1 * c2 * s3
        quaternion[0] = c1 * c2 * c3 - s1 * s2 * s3
        # w x y z
        return quaternion

    def quaternion_multiply(self, quaternionGripper, quaternionChange):

        w1, x1, y1, z1 = quaternionGripper
        w2, x2, y2, z2 = quaternionChange
        return np.array([-x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2,
                        x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2,
                        -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2,
                        x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2], dtype=np.float16)

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [[0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])
        duration_list = [3.0, 7.0, 10.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        # self.ask_confirmation(position_list)
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def go_to_point (self):
        pose_list = [
            self.pickupPoint,
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.06240, 0.48551, 0.30074),
                geometry_msgs.Quaternion(0.65186, 0.75830, -0.00706, 0.0010))
        ]

        duration_list = [3, 6]

        return pose_list, duration_list

    def idle_to_pick_up(self):
        pose_list = [
            self.idlePoint,
            self.pickupPoint
            ]

        duration_list = [1, 2]

        return pose_list, duration_list

    def new_quaternion_converter(self, quaternion):
        quaternion[2], quaternion[3] = quaternion[3], quaternion[2]
        quaternion[1], quaternion[2] = quaternion[2], quaternion[1]
        quaternion[0], quaternion[1] = quaternion[1], quaternion[0]

        return quaternion

    def new_angle_to_quaternions(self, angle):
        # atitude = z
        angle = angle * np.pi / 180
        c1 = 1
        c2 = np.cos(angle / 2)
        c3 = 1
        s1 = 0
        s2 = np.sin(angle / 2)
        s3 = 0

        quaternion = [0, 0, 0, 0]
        quaternion[3] = c1 * s2 * c3 - s1 * c2 * s3
        quaternion[0] = c1 * c2 * c3 - s1 * s2 * s3

        return quaternion

    def new_quaternion_multiply(self, quaternionGripper, quaternionChange):
        w0, x0, y0, z0 = quaternionGripper
        w1, x1, y1, z1 = quaternionChange
        return [
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0]



    # 45 0.70703 0.70703 -0.00705 -0.00043
    # -45

    def align_to_object(self, x_dist, y_dist):

        if self.iteration == 0 :
            self.my_list[0] = 0.06240 + x_dist
            self.my_list[1] = 0.48551 - y_dist

        else :
            self.my_list[0] = self.my_list[0] + x_dist
            self.my_list[1] = self.my_list[1] - y_dist

        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(self.my_list[0] - x_dist, self.my_list[1] + y_dist, 0.30074),
                geometry_msgs.Quaternion(0.9238536, 0.3826808, -0.0063537, -0.0030927)),  # Ready to grasp position,
            geometry_msgs.Pose(
                geometry_msgs.Vector3(self.my_list[0] , self.my_list[1] , 0.30074),
                geometry_msgs.Quaternion(0.9238536, 0.3826808, -0.0063537, -0.0030927))  # Ready to grasp position
        ]

        #temp. x : - 0.045 y : + 0.066

        # measured x-offeset : 0.035
        duration_list = [1, 2]


        self.iteration += 1

        return pose_list, duration_list



    def pick_up(self, z_dist, angle): # Add logic to compensate for Z-Axis offset from ground depending on object height.
        self.iteration = 0

        gripper_length = 0.1550
        x_offset = 0.045
        y_offset = 0.095

        newUR = self.new_quaternion_converter([0.9238536, 0.3826808, -0.0063537, -0.0030927])
        change = self.new_angle_to_quaternions(angle)
        quaternionur = self.new_quaternion_multiply(newUR, change)
        format_float = "{:.5f}".format(quaternionur[0])
        format_float2 = "{:.5f}".format(quaternionur[1])
        format_float3 = "{:.5f}".format(quaternionur[2])
        format_float4 = "{:.5f}".format(quaternionur[3])
        print(format_float, format_float2, format_float3, format_float4)


        self.my_list[2] = 0.30074 - z_dist + gripper_length
        print(self.my_list[2])
        print('z_dist' + str(z_dist))
        if self.my_list[2] < -0.21700:
            rospy.loginfo("Gripper will collide with ground")
            pose_list = [
                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0]), (self.my_list[1]), 0.30074),
                    geometry_msgs.Quaternion(0.9238536, 0.3826808, -0.0063537, -0.0030927))

                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0]), (self.my_list[1]), 0.30074),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3]))
                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0] - x_offset), (self.my_list[1] + y_offset), 0.30074),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3]))

                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0] - x_offset), (self.my_list[1] + y_offset), -0.21700),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3]))
                # Grasp near floor
            ]

            duration_list = [0.5, 1, 1.5, 2.5]

            return pose_list, duration_list, 0

        else :
            pose_list = [
                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0]), (self.my_list[1]), 0.30074),
                    geometry_msgs.Quaternion(0.9238536, 0.3826808, -0.0063537, -0.0030927))

                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0]), (self.my_list[1]), 0.30074),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3]))
                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0] - x_offset), (self.my_list[1] + y_offset), 0.30074),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3]))

                ,

                geometry_msgs.Pose(
                    geometry_msgs.Vector3((self.my_list[0] - x_offset), (self.my_list[1] + y_offset), (self.my_list[2])),
                    geometry_msgs.Quaternion(quaternionur[0], quaternionur[1], quaternionur[2], quaternionur[3])) # Grasp near floor
            ]

            duration_list = [0.5, 1, 1.5, 2.5]
            print(pose_list, 'pose_list')

            return pose_list, duration_list, 0




    def pick_up2(self):
        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(self.my_list[0], self.my_list[1], self.my_list[2]),
                geometry_msgs.Quaternion(self.gripperquarternion[0], self.gripperquarternion[1], self.gripperquarternion[2], self.gripperquarternion[3])),
            self.pickupPoint
        ]

        duration_list = [0.5, 2.5]

        return pose_list, duration_list


    def bin_selection(self, bin_number):
        if bin_number == 1:
            bin_point = geometry_msgs.Pose(
                geometry_msgs.Vector3(-0.18000, -0.30224, 0.88550),
                geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))
        elif bin_number == 2:
            bin_point = geometry_msgs.Pose(
                geometry_msgs.Vector3(0.00000, -0.30224, 0.88550),
                geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))
        elif bin_number == 3:
            bin_point = geometry_msgs.Pose(
                geometry_msgs.Vector3(0.18000, -0.30224, 0.88550),
                geometry_msgs.Quaternion(0.2688929, -0.6626699, 0.6472444, 0.2638937))


        return bin_point

    def bin_drop_selection (self, bin_number):
        if bin_number == 1:
            bin_point = geometry_msgs.Pose(
            geometry_msgs.Vector3(-0.18000, -0.37870, 0.78191),
            geometry_msgs.Quaternion(0.3320563, -0.8176629, 0.4353422, 0.1778851))
        elif bin_number == 2:
            bin_point = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.00000, -0.37870, 0.78191),
            geometry_msgs.Quaternion(0.3320563, -0.8176629, 0.4353422, 0.1778851))
        elif bin_number == 3:
            bin_point = geometry_msgs.Pose(
            geometry_msgs.Vector3(0.18000, -0.37870, 0.78191),
            geometry_msgs.Quaternion(0.3320563, -0.8176629, 0.4353422, 0.1778851))


        return bin_point


    def pickup_to_idle(self):
        pose_list = [
            self.pickupPoint,
            self.idlePoint
        ]
        duration_list = [2, 4]

        return pose_list, duration_list


    def pick_up_to_bin(self, bin_number):  # Remember to set way point in secure position! IMPORTANT

        time_to_bin = 6.0

        duration_list = [1, 2, 4, 5, time_to_bin, time_to_bin + 0.5]

        if bin_number==1:

            duration_list[4] = time_to_bin - 0.5

            pose_list = [
                self.pickupPoint_low,
                self.pickupPoint,  # Ready to grasp position
                self.wayPoint,  # safe waypoint
                self.bin_selection(1),
                self.bin_drop_selection(bin_number)

                # self.binApproachPoint,

            ]

        elif bin_number==2:

            time_to_bin= 5

            pose_list = [
                self.pickupPoint_low,
                self.pickupPoint,  # Ready to grasp position
                self.wayPoint,  # safe waypoint
                self.bin_selection(1),
                self.bin_selection(bin_number),
                self.bin_drop_selection(bin_number)
                # self.binApproachPoint,

            ]

        elif bin_number==3:

            time_to_bin= 5.5
            pose_list = [
                self.pickupPoint_low,
                self.pickupPoint,  # Ready to grasp position
                self.wayPoint,  # safe waypoint
                self.bin_selection(1),
                self.bin_selection(bin_number),
                self.bin_drop_selection(bin_number)
                # self.binApproachPoint,

            ]

        else:
            time_to_bin = 6.5 # If something is wrong, set time to 6.5 (Form of Debug)
            pose_list = [
                self.pickupPoint,  # Ready to grasp position
                self.wayPoint,  # safe waypoint
                self.bin_selection(1),
                self.bin_drop_selection(bin_number)
                # self.binApproachPoint,

            ]



        return pose_list, duration_list


    def bin_to_idle(self, bin_number):
        bin_to_bin_time=0

        duration_list = [0.5, 2.5, 3.5, 4.5]

        if bin_number == 1:
            pose_list = [
                self.bin_selection(bin_number),
                self.wayPointReturn,  # safe waypoint
                self.safeIdlePoint,  # Safe return to idle from waypoint
                self.pickupPoint

            ]


        elif bin_number == 2:

            pose_list = [
                self.bin_selection(bin_number),
                self.bin_selection(1),
                self.wayPointReturn,  # safe waypoint
                self.safeIdlePoint,  # Safe return to idle from waypoint
                self.pickupPoint
                # self.binApproachPoint,

            ]

            bin_to_bin_time = 1

            duration_list.insert(1, bin_to_bin_time)

        elif bin_number == 3:

            pose_list = [
                self.bin_selection(bin_number),
                self.bin_selection(1),
                self.wayPointReturn,  # safe waypoint
                self.safeIdlePoint,  # Safe return to idle from waypoint
                self.pickupPoint
                # self.binApproachPoint,

            ]


            bin_to_bin_time = 1.5

            duration_list.insert(1, bin_to_bin_time)

        else:

            pose_list = [
                self.bin_selection(1),
                self.wayPointReturn,  # safe waypoint
                self.safeIdlePoint,  # Safe return to idle from waypoint
                self.pickupPoint
                # self.binApproachPoint,

            ]
            duration_list = [2, 3, 4.5, 6]

        print(duration_list)
        return pose_list, duration_list

    def send_cartesian_trajectory(self, pose_list, duration_list):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # The following list are arbitrary positions
        # Change to your own needs if desired

        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        # self.ask_confirmation(pose_list)
        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    ###############################################################################################
    #                                                                                             #
    # Methods defined below are for the sake of safety / flexibility of this demo script only.    #
    # If you just want to copy the relevant parts to make your own motion script you don't have   #
    # to use / copy all the functions below.                                                       #
    #                                                                                             #
    ###############################################################################################

    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        rospy.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == "y"
        if not confirmed:
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def choose_controller(self):
        """Ask the user to select the desired controller from the available list."""
        rospy.loginfo("Available trajectory controllers:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (joint-based): {}".format(index, name))
        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (Cartesian): {}".format(index + len(JOINT_TRAJECTORY_CONTROLLERS), name))
        choice = -1
        while choice < 0:
            input_str = 7

            try:
                choice = int("7")
                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
                        CARTESIAN_TRAJECTORY_CONTROLLERS
                ):
                    rospy.loginfo(
                        "{} not inside the list of options. "
                        "Please enter a valid index from the list above.".format(choice)
                    )
                    choice = -1
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice - len(JOINT_TRAJECTORY_CONTROLLERS)
            ]
        return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
                JOINT_TRAJECTORY_CONTROLLERS
                + CARTESIAN_TRAJECTORY_CONTROLLERS
                + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)



def URmain():
    client = TrajectoryClient()
    gripper = GripperControl()
    align = URAlignment()
    time.sleep(1)

    attempt = 0

    x_dist_thresh = 0.005
    y_dist_thresh = 0.005
    x_dist = 0
    y_dist = 0

    z_dist = 0

    angle = 0

    chosen_bin = 2

    # The controller choice is obviously not required to move the robot. It is a part of this demo
    # script in order to show all available trajectory controllers.
    trajectory_type = client.choose_controller()
    if trajectory_type == "joint_based":
        client.send_joint_trajectory()
    elif trajectory_type == "cartesian":
        '''
        pose_list, duration_list = client.pickup_to_idle()
        client.send_cartesian_trajectory(pose_list, duration_list)
        '''
        gripper.publisher(0)
        #time.sleep(0.5)
        pose_list, duration_list = client.idle_to_pick_up()
        client.send_cartesian_trajectory(pose_list, duration_list)

        while align.get_object_position() != -1:

            (x_dist, y_dist) = align.get_object_position()

            while abs(x_dist) > abs(x_dist_thresh) or abs(y_dist) > abs(y_dist_thresh) and align.get_object_position() != -1 and attempt < 4:
                rospy.loginfo(attempt)
                pose_list, duration_list = client.align_to_object(x_dist, y_dist)
                client.send_cartesian_trajectory(pose_list, duration_list)

                #run check alignment values function that returns values x_dist and y_dist
                (x_dist, y_dist) = align.get_object_position()
                attempt += 1


            rospy.loginfo("Loop er brudt")

            time.sleep(1)

            (angle, z_dist, bin_number) = align.get_angle_depth_object()

            rospy.loginfo("vinkel mm. er fundet")



            pose_list, duration_list, pickup_status = client.pick_up(z_dist, angle) #700 giver 3 cm fra jorden
            client.send_cartesian_trajectory(pose_list, duration_list)
            #time.sleep(1)

            attempt = 0

            if (pickup_status == -1):
                break

            gripper.publisher(255)
            time.sleep(1)



            pose_list, duration_list = client.pick_up_to_bin(bin_number)
            client.send_cartesian_trajectory(pose_list, duration_list)
            time.sleep(1)



            gripper.publisher(0)
            time.sleep(0.5)



            pose_list, duration_list = client.bin_to_idle(bin_number) #actually going to pickup
            client.send_cartesian_trajectory(pose_list, duration_list)

        pose_list, duration_list = client.pickup_to_idle()
        client.send_cartesian_trajectory(pose_list, duration_list)



    else:
        raise ValueError(
            "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(
                trajectory_type
            )
        )

#URmain()
