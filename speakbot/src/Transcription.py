#!/usr/bin/env python3

# Transcription.py
#
# Author: David Jeffreys (P2655839)
#
# Acquires audio input from a microphone peripheral through PyAudio, transcribes the audio file utilising Whisper local 'base' model, and processes the transcribed text within a DetermineAction class
# (toolkit), enabling generation of output parameters that are deferenced effectively by the altered_teleop_twist_keyboard.py Python module. End-effector manipulation, mobile base localization and
# navigation and object identification (where necessary) are constructed through 

import rospy
import pyaudio
import wave
import sys
import whisper
import copy
import re
import os
import io
import actionlib
import moveit_commander
import numpy as np
import threading
import statistics as stats

from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal 
from sensor_msgs.msg import LaserScan

# import openai

# !!!!!!!!!! REMINDER TO UPDATE PACKAGE.XML FOR THE ABOVE DEPENDENCIES !!!!!!!!!!  #

OPENAT_API_KEY = os.getenv("OPENAI_API_KEY")
GLOBAL_FRAME = "world"
GROUP_ARM = "arm"
GROUP_GRIPPER = "gripper"
HOME_POSE_CONFIG = [0.0, -1.0, 0.3, 0.7]
PLACE_POSE_CONFIG = [1.575, 0.0, 0.0, 0.0]

actionVerb = {
    "Get": ("Bring", "Find", "Fetch", "Give", "Hand", "Grab", "Pick", "pick")
}
actionNoun_info = {                                 # Dictionary values may need to be altered into a list instead of a tuple to allow for modification in a dynamic map.
    "red block"          : [], 
    "green block"        : [],
    "blue block"         : [],
    # "red box_orient"        : [], 
    # "green box__orient"     : [],
    # "blue box_orient"       : [],
}

actionNoun = {
    "red block"     : ["red box", "red block", "red object"],
    "blue block"     : ["blue box", "blue block", "blue object"],
    "green block"     : ["green box", "green block", "green object"]
}

# base_actionDirective = ["Turn right", "Turn left", "Go forward", "Go back", "Stop"]
# base_inputMovement = ("l", "j" , "i" , ",", "a")
# manipulator_jointpose = {
#     "right"     : (-1.570791, 1.131916, -0.659249, 1.084142),
#     "left"      : (1.570791, 1.131916, -0.659249, 1.084142),
#     "front"     : (0.0, 1.032956, -0.663504, 1.198832),
#     "transit"   : (0.0,0.0,0.0,1.543779),
#     "home"      : (0.0,0.0,0.0,0.0)
# }

class TranscribeAudio:
    def __init__(self):
        pass

    def RecordAudio(self):
        print("What would you like me to do?")

# Parameter setup for audio manipulation
        
        FRAMES_PER_BUFFER = 3200
        FORMAT = pyaudio.paInt16                                    # Suitable bit depth for processing audio files - matches audio bit depth used in CDs, without larger memory usage of 24/32 bit depths.
        CHANNELS = 1                                                # Number of channels irrelevant in this application, simply set to mono.
        RATE = 24000
        seconds = 5

        p = pyaudio.PyAudio()

        stream = p.open(                                            # Initialisation of the parameters within the 'p' object of the PyAudio class.
            format = FORMAT,
            channels = CHANNELS,
            rate = RATE,
            input = True,
            frames_per_buffer=FRAMES_PER_BUFFER
        )

        obj = wave.open("Output.wav", "wb")                         # Create an object to save the .wav sample in a wave file. The file is set to 'write binary' for the allocatation of th frames list into the file.
        obj.setnchannels(CHANNELS)
        obj.setsampwidth(p.get_sample_size(FORMAT))
        obj.setframerate(RATE)

        for _ in range (0, int(RATE/FRAMES_PER_BUFFER*seconds)):    # Iterate through the frames list, filling up each element every second.
            data = stream.read(FRAMES_PER_BUFFER)                   # Number of frames per buffer stated on line 4 read per each iteration of the for loop. Returns a byte value.
            obj.writeframes(data)

        obj.close() 
        stream.stop_stream()
        stream.close()
        p.terminate()                                               # Releases all the PortAudio resources.
        TransAudio.ConvertAudio()

    def ConvertAudio(self):
        # result = openai.audio.transcriptions.create(model="whisper-1",file=("Output.wav", "rb"))
        self.model = whisper_model
        result = self.model.transcribe("Output.wav")
        result = result["text"]
        result = (result.replace(",",""))
        #result = result.title()
        result = result.strip(" .")
        result_period = tuple(result.split())
        #result = result.split()

        print(result_period)
        Actual_Task.actionMovement(result, result_period)
        
        # try:
        #     hotword = re.search("\AHey speaker", result)
        #     print(hotword)

        #     if hotword.group() == "Hey speaker":
        #         #Actual_Task.setObjectInfo(result, manipulator_result)
        #         #Actual_Task.actionMovement(result, manipulator_result)
        #         print("LISTENING")
        # except:
        #     print("NO HOTWORD DETECTED")
        #     os.remove("Output.wav")
        #     pass
            
class DetermineAction:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self.targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)     

    def setObjectInfo(self):                                               # Name of the object to be passed as argument once functionality is confirmed
        redbox_request = GetModelStateRequest()
        bluebox_request = GetModelStateRequest()
        greenbox_request = GetModelStateRequest()
        redbox_response = GetModelStateResponse()
        bluebox_response = GetModelStateResponse()
        greenbox_response = GetModelStateResponse()
        redbox_request.model_name = "red_box"
        bluebox_request.model_name = "blue_box"
        greenbox_request.model_name = "green_box"
        redbox_request.relative_entity_name = "world"
        bluebox_request.relative_entity_name = "world"
        greenbox_request.relative_entity_name = "world"
        redbox_response = self.targetobj_1(redbox_request)
        bluebox_response = self.targetobj_1(bluebox_request)
        greenbox_response = self.targetobj_1(greenbox_request)
        redbox_pose = redbox_response.pose.position
        bluebox_pose = bluebox_response.pose.position
        greenbox_pose = greenbox_response.pose.position
        redbox_orientation = redbox_response.pose.orientation
        bluebox_orientation = bluebox_response.pose.orientation
        greenbox_orientation = greenbox_response.pose.orientation

        actionNoun_info["red block"] = [
            round(redbox_pose.x, 6),
            round(redbox_pose.y, 6),
            round(redbox_orientation.z, 6)
        ]

        actionNoun_info["blue block"] = [
            round(bluebox_pose.x, 6),
            round(bluebox_pose.y, 6),
            round(bluebox_orientation.z, 6)
        ]

        actionNoun_info["green block"] = [
            round(greenbox_pose.x, 6),
            round(greenbox_pose.y, 6),
            round(greenbox_orientation.z, 6)
        ]

    def actionMovement(self, result, result_period):
        actual_request = actionVerb.get("Get")

        for i in result_period:
            if i in actual_request:
                try:
                    matched_object = re.search(r"(\bred|\bblue|\bgreen)\s(\bblock)", result)
                except Exception as error:
                    rospy.logerr(f"Invalid arm location request: {error}")
                    break
                else:
                    matched_object = matched_object.group()

                    object_offset = -0.23 if actionNoun_info[matched_object][0] > 0 else 0.23

                    if matched_object in actionNoun.keys():
                        wafflepose_msg.pose.position.x = actionNoun_info[matched_object][0] + object_offset  
                        wafflepose_msg.pose.position.y = actionNoun_info[matched_object][1] + 0.04
                        wafflepose_msg.pose.position.z = 0.0
                        wafflepose_msg.pose.orientation.x = 0.0
                        wafflepose_msg.pose.orientation.y = 0.0
                        wafflepose_msg.pose.orientation.z = actionNoun_info[matched_object][2] + 0.95
                        wafflepose_msg.pose.orientation.w = 1.0
                        wafflepose_msg.header.stamp = rospy.Time.now()
                        wafflepose_msg.header.frame_id = "map"

                        wafflepose = MoveBaseGoal()
                        wafflepose.target_pose = wafflepose_msg
                        status = base_client.send_goal_and_wait(wafflepose)
                        rospy.loginfo(f"Goal achieved: {status}")
                        Control_Arm.grabObject(matched_object)

class ControlTrajectory:
    def __init__(self):
        self.move_arm = MoveGroupCommander(GROUP_ARM)
        self.move_gripper = MoveGroupCommander(GROUP_GRIPPER)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.move_gripper.set_pose_reference_frame(GLOBAL_FRAME)
        self.move_arm.set_pose_reference_frame(GLOBAL_FRAME)
        self.move_arm.set_goal_position_tolerance(0.005)
        #self.move_arm.set_goal_joint_tolerance(0.005)

        self.move_arm.go(HOME_POSE_CONFIG)
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
        self.open_gripper()
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()

        rospy.loginfo("Init Complete")

    def open_gripper(self):
        # open_init = rospy.Time.now()
        gripper_open = JointTrajectory()
        gripper_open_points = JointTrajectoryPoint()
        gripper_open.joint_names = ["gripper"]
        gripper_open_points.positions = [0.015]
        gripper_open_points.time_from_start = rospy.Duration(1.0)
        gripper_open.points = [gripper_open_points]
        self.gripper_pub.publish(gripper_open)
        rospy.sleep(2.0)

    def close_gripper(self):
        # grasp_init = rospy.Time.now()
        gripper_close = JointTrajectory()
        gripper_close_points = JointTrajectoryPoint()
        gripper_close.joint_names = ["gripper"]
        gripper_close_points.positions = [0.0]
        # gripper_close_points.effort = [0.8]
        gripper_close_points.time_from_start = rospy.Duration(1.0)
        gripper_close.points = [gripper_close_points]
        self.gripper_pub.publish(gripper_close)

    def grabObject(self, item):
        Actual_Task.setObjectInfo()
        waypoints = []
        goal_status = False

        x_pose = actionNoun_info[item][0]
        y_pose = actionNoun_info[item][1]
        z_pose = actionNoun_info[item][2] 

        while goal_status is not True:
            EE_pose = self.move_arm.get_current_pose().pose
            EE_pose_x = EE_pose.position.x
            EE_pose_y = EE_pose.position.y
            EE_pose_z = EE_pose.position.z
            x_pose_error = round((x_pose-EE_pose_x), 4)
            y_pose_error = round((y_pose-EE_pose_y), 4)
            z_pose_error = round(((z_pose-0.025)-EE_pose_z), 4)

            EE_pose.position.x += (x_pose_error*0.05)
            EE_pose.position.y += (y_pose_error*0.05)
            EE_pose.position.z += (z_pose_error*0.05)
            waypoints.append(copy.deepcopy(EE_pose))

            (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, 0.01)

            self.move_arm.execute(plan, wait=False)
            waypoints.clear()

            if abs(x_pose_error < 0.01) and abs(y_pose_error < 0.01) and abs(z_pose_error <= 0.01):
                print(x_pose_error)
                print(y_pose_error)
                print(z_pose_error)
                # move_arm.stop()
                # print("CLOSING")
                # close_gripper()
                # rospy.sleep(2.0)
                # move_arm.go(place_pose, wait=True)  
                # rospy.sleep(2.0)
                # open_gripper()
                # rospy.sleep(2.0)
                # move_arm.go(home_pose, wait=True)
                # move_arm.clear_pose_targets()
                # goal_status = True
                break

            if rospy.is_shutdown():
                break

            rospy.sleep(0.04)

class TaskThreader:
    def __init__(self):
        self.move_base_status = 0
        pass

    def acquire_base_status(self):
        self.move_base_status = base_client.get_state()
    
    def start_SpeakBot(self):
        while not rospy.is_shutdown():
            Actual_Task.setObjectInfo() 
            TransAudio.RecordAudio()
            TransAudio.ConvertAudio()

    def start_ProximitySens(self):
        rospy.Subscriber("/scan", LaserScan, SpeakBot_Thread.process_Scanner)
        rospy.spin()

    def process_Scanner(self, scan):
        SpeakBot_Thread.acquire_base_status()
        lowest_val = sorted(scan.ranges)[:5]
        lowest_val = round((stats.mean(lowest_val)), 3)

        print(self.move_base_status)
        
        if lowest_val < 0.4 and self.move_base_status == 0:
            print("OBJECT DETECTED WHILST STATIC")
        else:
            pass

if __name__ == '__main__':
    rospy.init_node("transcribe")
    moveit_commander.roscpp_initialize(sys.argv)
    whisper_model = whisper.load_model("base.en")
    TransAudio = TranscribeAudio()
    Actual_Task = DetermineAction()
    Control_Arm = ControlTrajectory()
    SpeakBot_Thread = TaskThreader()
    
    wafflepose_msg = PoseStamped()
    targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    try:
        base_client.wait_for_server()
        rospy.loginfo("Move base server connected successfully")
    except ConnectionRefusedError as e:
        rospy.logerr(f"Server failed: {e}")

    try:
        targetobj_1.wait_for_service()
        rospy.loginfo("Object localization service live")
    except ConnectionRefusedError as e:
        rospy.logerr(f"Service failed: {e}")

    SpeakBot = threading.Thread(target=SpeakBot_Thread.start_SpeakBot)
    Proximity_Sensor = threading.Thread(target=SpeakBot_Thread.start_ProximitySens)

    SpeakBot.start()
    Proximity_Sensor.setDaemon(True)
    Proximity_Sensor.start()