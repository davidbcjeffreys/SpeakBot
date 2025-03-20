#!/usr/bin/env python3

# SpeakBot.py
#
# Author: David Jeffreys (P2655839)
#
# Acquires audio input from a microphone peripheral through PyAudio, transcribes the audio file utilising Whisper local 'base' model, and processes the transcribed text within a DetermineAction class
# (toolkit), enabling generation of output parameters that are deferenced effectively by the altered_teleop_twist_keyboard.py Python module. End-effector pose manipulation and mobile base localization 
# and navigation are undertaken through RRT Connect OMPL and AMCL SLAM respectively utilising move_it package.

import rospy
import pyaudio
import wave
import sys
import whisper
import copy
import re
import numpy as np
import actionlib
import moveit_commander
import threading
import statistics as stats
import tf
from moveit_commander import MoveGroupCommander
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import LaserScan

# import openai

# !!!!!!!!!! REMINDER TO UPDATE PACKAGE.XML FOR THE ABOVE DEPENDENCIES !!!!!!!!!!  #

# OPENAT_API_KEY = os.getenv("OPENAI_API_KEY")

# move_group 'group's and global frame declarations
GLOBAL_FRAME = "world"
GROUP_ARM = "arm"
GROUP_GRIPPER = "gripper"

# move_group trajectory waypoint parameters. EE chain and link to base link offsets also declared for
# navigation of mobile base.
EE_TO_GRIPPERLINK_DIST = 0.075
LINK1_TO_BASEINK_DIST = 0.091
GOAL_POSITION_TOLERANCE = 0.005
ARM_TRAJECTORY_WAYPOINT_STEP = 0.01

# Gripper states
ENTRY = 0
ORIENT = 1
DESCEND = 2
OBTAIN = 3
ASCEND = 4
RELEASE = 5

# Joint configurations
HOME_CONFIG = [0.0, -1.0, 0.3, 0.7]
RELEASE_CONFIG = [1.575, 0.0, 0.0, 0.0]
TRANSIT_CONFIG = [0.0, -0.47, 0.06, 1.97]
GRIPPER_CLOSE = [-0.01, 0.0]
GRIPPER_OPEN = [0.01, 0.0]

actionVerb = {
    "Get": ("Bring", "Find", "Fetch", "Give", "Hand", "Grab", "Pick", "bring", "find", "fetch", "give", "hand", "grab")
}
actionNoun_info = {                                                 # Dictionary values may need to be altered into a list instead of a tuple to allow for modification in a dynamic map.
    "red block"          : [], 
    "green block"        : [],
    "blue block"         : [],
}

actionNoun = {
    "red block"     : ["red box", "red block", "red object"],
    "blue block"     : ["blue box", "blue block", "blue object"],
    "green block"     : ["green box", "green block", "green object"]
}

class GetInstruction:
    def __init__(self):
        self.hotword_detected = False
        self.find_action = False

    def RecordAudio(self):
        print("What would you like me to do?")

# Parameter setup for audio manipulation
        
        FORMAT = pyaudio.paInt16                                    # Suitable bit depth for processing audio files - matches audio bit depth used in CDs, without larger memory usage of 24/32 bit depths.
        CHANNELS = 1                                                # Number of channels irrelevant in this application, simply set to mono.
        if self.hotword_detected:
            seconds = 5
            RATE = 24000
            FRAMES_PER_BUFFER = 3200
            self.hotword_detected = False
            self.find_action = True
        else:
            seconds = 2
            RATE = 24000
            FRAMES_PER_BUFFER = 3200

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
        UserCommand.ConvertAudio()

    def ConvertAudio(self):
        result = whisper_model.transcribe("Output.wav", no_speech_threshold=0.75, logprob_threshold=-3.0)
        result = result["text"]
        result = (result.replace(",",""))
        result = result.strip(" .")
        result_period = tuple(result.split())

        if self.find_action:
            GoalAction.movetotarget(result, result_period)      
        try:
            hotword = re.search("\AHey speaker", result)
            if hotword.group() == "Hey speaker":
                print("LISTENING")
                self.hotword_detected = True
        except:
            pass
            
class Navigate:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self.targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.amcl_reading = PoseWithCovariance()
        self.wafflepose_locate = PoseStamped()
        self.amcl_poses = [0.0, 0.0]
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.home_iteration = False

        # Command string and command tuple for testing without audio
        # self.result_string = "Pick up the blue block"
        # self.result_tuple = ("Pick", "up", "the", "blue", "block")

    def setObjectInfo(self):
        
        # Name of the object to be passed as argument once functionality is confirmed.
        # Naturally this would happen once the model has undertaken initial SLAM, with
        # the poses of the target objects recorded from the model.

        # Object definitions. Manually filled in, but upon user command dataset broadening
        # past the pre-filled set, the new objects will be appended to the list.
        target_objects = ["red_box", "blue_box", "green_box"]
        responses = {}

        # Acquiring information for each object within the dataset.
        for target in target_objects:
            request = GetModelStateRequest()
            response = GetModelStateResponse()
            
            request.model_name = target
            request.relative_entity_name = "world"
            
            # Get the model state
            response = self.targetobj_1(request)
            
            # Store the response (pose and orientation) in the dictionary
            responses[target] = {
                'pose': response.pose.position,
                'orientation': response.pose.orientation
            }

        # Access the pose and orientation for each model
        redbox_pose = responses["red_box"]['pose']
        bluebox_pose = responses["blue_box"]['pose']
        greenbox_pose = responses["green_box"]['pose']

        redbox_orientation = responses["red_box"]['orientation']
        bluebox_orientation = responses["blue_box"]['orientation']
        greenbox_orientation = responses["green_box"]['orientation']

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

    def movetotarget(self, result, result_period):
        actual_request = actionVerb.get("Get")
        # Acquire initial home pose position for object returns upon initialization
        if self.home_iteration is False:
            self.home_pose = self.amcl_poses
            print(self.home_pose)
            self.home_iteration = True
        for i in result_period:
            if i in actual_request:
                try:
                    self.matched_object = re.search(r"(\bred|\bblue|\bgreen)\s(\bblock)", result)
                except Exception as error:
                    rospy.logerr(f"Invalid arm location request: {error}")
                    break
                if self.matched_object:
                    self.matched_object = self.matched_object.group()
                    rospy.sleep(1.5)                                        # The /amcl_pose topic publishes at a low frequency of around 0.66Hz. Unless there is a way to adjust this publishing frequency, a rudimentary 'buffer' must be implemented - a timer to allow the callback function to acquire the value.
                    if self.matched_object in actionNoun.keys():
                        self.orient_vals = self.base_orient()
                        self.wafflepose_locate.pose.position.x = actionNoun_info[self.matched_object][0] + self.orient_vals[0]
                        self.wafflepose_locate.pose.position.y = actionNoun_info[self.matched_object][1] + self.y_offset
                        self.wafflepose_locate.pose.orientation.z = self.orient_vals[1]
                        self.wafflepose_locate.pose.orientation.w = self.orient_vals[2]
                        self.wafflepose_locate.header.stamp = rospy.Time.now()
                        self.wafflepose_locate.header.frame_id = "world"

                        self.wafflepose = MoveBaseGoal()
                        self.wafflepose.target_pose = self.wafflepose_locate
                        base_client.send_goal(self.wafflepose)
                        if base_client.wait_for_result():
                            base_client.cancel_goal()
                            self.wafflepose.target_pose = None
                        print("TARGET REACHED")
                        #ControlArm.grabObject(self.matched_object)
                        self.movetohome()

    def movetohome(self):
        self.home_goal = MoveBaseGoal()
        self.home_target = PoseStamped()
        self.home_target.header.frame_id = "world"
        self.home_target.header.stamp = rospy.Time.now()
        self.home_target.pose.position.x = self.home_pose[0]
        self.home_target.pose.position.y = self.home_pose[1]
        self.home_target.pose.orientation.w = 1.0
        self.home_goal.target_pose = self.home_target
        base_client.send_goal_and_wait(self.home_goal)
        if base_client.wait_for_result():
            base_client.cancel_goal()
            self.wafflepose.target_pose = None
        print("TASK COMPLETE")

    def base_orient(self):
        if self.x_error > 0:
            self.x_location = LINK1_TO_BASEINK_DIST
            self.z_orient = 0.0
            self.w_orient = 1.0
        else:
            self.x_location = -LINK1_TO_BASEINK_DIST
            self.z_orient = -0.7
            self.w_orient = 0.0
        
        return self.x_location, self.z_orient, self.w_orient
    
    def get_AMCL_pose(self, lidar_val):
        # Calibrates the AMCL position of the Waffle Pi.
        while self.home_iteration is False:
            amcl_reading = lidar_val.pose
            self.amcl_poses[0] = amcl_reading.pose.position.x
            self.amcl_poses[1] = amcl_reading.pose.position.y

        # Error checking is undertaken between the Pi pose and the assumed 
        # object pose within the global frame, setting an offset for the
        # target object pose.
        if hasattr(self, "matched_object"):
            self.x_error = (actionNoun_info[GoalAction.matched_object][0]-self.amcl_poses[0])
            self.y_error = (actionNoun_info[GoalAction.matched_object][1]-self.amcl_poses[1])
            self.y_offset = -0.275 if self.y_error > 0.0 else 0.275
        rospy.sleep(0.25)
            
class ControlTrajectory:
    def __init__(self):
        self.move_arm = MoveGroupCommander(GROUP_ARM)
        self.move_gripper = MoveGroupCommander(GROUP_GRIPPER)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.move_gripper.set_pose_reference_frame(GLOBAL_FRAME)
        self.move_arm.set_pose_reference_frame(GLOBAL_FRAME)
        self.move_arm.set_goal_position_tolerance(GOAL_POSITION_TOLERANCE)
        self.move_arm.go(HOME_CONFIG)
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
        self.move_gripper.go(GRIPPER_OPEN, wait=False)
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        self.listener = tf.TransformListener()

        rospy.loginfo("Init Complete")

    def transform_baseframe(self, pose):
        self.listener.waitForTransform("world", "base_footprint", rospy.Time(), rospy.Duration(4.0))
        self.pose_pretransform = PoseStamped()
        self.pose_pretransform.header.frame_id = "base_footprint"
        self.pose_pretransform.header.stamp = rospy.Time(0)
        self.pose_pretransform.pose = pose
        pose_world = self.listener.transformPose("world", self.pose_pretransform)

        return pose_world

    def grabObject(self, item):
        waypoints = []
        goal_status = False
        state = ENTRY

        GoalAction.setObjectInfo()
        x_pose = actionNoun_info[item][0]
        y_pose = actionNoun_info[item][1]
        z_pose = actionNoun_info[item][2]

        while goal_status is not True:
            while state is ENTRY:
                transformed_pose_entry = self.transform_baseframe(self.move_arm.get_current_pose().pose)
                transformed_pose_entry = transformed_pose_entry.pose
                x_pose_error = round((x_pose-transformed_pose_entry.position.x), 4)
                y_pose_error = round(((y_pose+EE_TO_GRIPPERLINK_DIST)-transformed_pose_entry.position.y), 4)
                transformed_pose_entry.position.x += x_pose_error
                transformed_pose_entry.position.y += y_pose_error
                waypoints.append(copy.deepcopy(transformed_pose_entry))
                
                if abs(x_pose_error) < 0.005:
                    state = ORIENT
                    self.move_arm.stop()
                    self.move_arm.clear_pose_targets()
                    rospy.sleep(2.0)
                    break

                if rospy.is_shutdown():
                    break

                (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, ARM_TRAJECTORY_WAYPOINT_STEP)
                self.move_arm.execute(plan, wait=False)
                waypoints.clear()
                rospy.sleep(0.03)
            
            if state is ORIENT:
                place_config = self.move_arm.get_current_joint_values()
                place_config[3] = 1.575
                self.move_arm.go(place_config, wait=True)
                state = DESCEND
                waypoints.clear()

            while state is DESCEND:
                transformed_pose_descend = self.transform_baseframe(self.move_arm.get_current_pose().pose)
                z_pose_error = (round((z_pose-transformed_pose_descend.pose.position.z), 4)) + 0.01
                print(z_pose_error)
                transformed_pose_descend.pose.position.z += (z_pose_error*0.5)
                waypoints.append(copy.deepcopy(transformed_pose_descend))
                
                if abs(z_pose_error) < 0.02:
                    print("LOCATED")
                    state = OBTAIN
                    break

                if rospy.is_shutdown():
                    break

                (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, ARM_TRAJECTORY_WAYPOINT_STEP)
                self.move_arm.execute(plan, wait=False)
                waypoints.clear()
                rospy.sleep(0.03)
            
            if state is OBTAIN:
                self.move_arm.stop()
                print("CLOSING")
                self.move_gripper.go(GRIPPER_CLOSE, wait=True)
                rospy.sleep(2.0)
                state = ASCEND

            if state is ASCEND:
                self.move_arm.go(TRANSIT_CONFIG, wait=True)  
                rospy.sleep(2.0)
                print("OBJECT ACQUIRED")
                Navigate.movetohome()
                break


            if rospy.is_shutdown():
                break

class TaskThreader:
    def __init__(self):
        pass

    def acquire_base_status(self):
        self.move_base_status = base_client.get_state()
    
    def start_SpeakBot(self):
        # Start Speakbot
        while not rospy.is_shutdown():
            GoalAction.setObjectInfo()
            # GoalAction.movetotarget(GoalAction.result_string, GoalAction.result_tuple)
            UserCommand.RecordAudio()

    def start_ProximitySens(self):
        rospy.Subscriber("/scan", LaserScan, SpeakBot_Thread.process_Scanner)
        rospy.spin()

    def process_Scanner(self, scan):
        SpeakBot_Thread.acquire_base_status()
        lowest_val = sorted(scan.ranges)[:5]
        lowest_val = round((stats.mean(lowest_val)), 3)
        
        if lowest_val < 0.4 and self.move_base_status == 0:
            print("OBJECT DETECTED WHILST STATIC")
        else:
            pass

if __name__ == '__main__':
    rospy.init_node("transcribe")
    moveit_commander.roscpp_initialize(sys.argv)
    whisper_model = whisper.load_model("base.en")
    UserCommand = GetInstruction()
    GoalAction = Navigate()
    ControlArm = ControlTrajectory()
    SpeakBot_Thread = TaskThreader()
    
    targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, GoalAction.get_AMCL_pose, queue_size=1)    
    try:
        base_client.wait_for_server(timeout=rospy.Duration(5.0))
        rospy.loginfo("Move base server connected successfully")
    except ConnectionRefusedError as e:
        rospy.logerr(f"Server failed: {e}")

    try:
        targetobj_1.wait_for_service(timeout=rospy.Duration(5.0))
        rospy.loginfo("Object localization service live")
    except ConnectionRefusedError as e:
        rospy.logerr(f"Service failed: {e}")

    SpeakBot = threading.Thread(target=SpeakBot_Thread.start_SpeakBot)
    Proximity_Sensor = threading.Thread(target=SpeakBot_Thread.start_ProximitySens)

    SpeakBot.start()
    Proximity_Sensor.setDaemon(True)
    Proximity_Sensor.start()