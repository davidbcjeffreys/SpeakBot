#!/usr/bin/env python3
#
# SpeakBot.py
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1
#
# Acquires audio input from a microphone peripheral through PyAudio, transcribes the audio file utilising Whisper local 'base' model, and processes the transcribed text within a DetermineAction class
# (toolkit), enabling generation of output parameters that are deferenced effectively by the altered_teleop_twist_keyboard.py Python module. End-effector pose manipulation and mobile base localization 
# and navigation are undertaken through RRT Connect OMPL and AMCL SLAM respectively utilising move_it package.

import rospy
import pyaudio
import wave
import sys
import copy
import re
import actionlib
import moveit_commander
import threading
import statistics as stats
import tf
import os
import openai
from dynamic_reconfigure.client import Client
from moveit_commander import MoveGroupCommander
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import LaserScan

# !!!!!!!!!! REMINDER TO UPDATE PACKAGE.XML FOR THE ABOVE DEPENDENCIES !!!!!!!!!!  #

#OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# move_group group's and global frame declarations
GLOBAL_FRAME = "map"
NAVIGATION_FRAME = "odom"
GROUP_ARM = "arm"
GROUP_GRIPPER = "gripper"

# move_group trajectory waypoint parameters. EE chain and link to base link offsets also declared for
# navigation of mobile base.
EE_TO_GRIPPERLINK_DIST = 0.05
LINK1_TO_BASEINK_DIST = 0.091
GOAL_POSITION_TOLERANCE = 0.005
ARM_TRAJECTORY_WAYPOINT_STEP = 0.01

# Gripper states
PREP = 0
ENTRY = 1
ORIENT = 2
DESCEND = 3
OBTAIN = 4
ASCEND = 5
RETURN = 6
RELEASE = 7

# Joint configurations
HOME_CONFIG = [0.0, -1.0, 0.3, 0.7]
RIGHT_PICKUP = [-1.575, -1.0, 0.3, 0.7]
LEFT_PICKUP = [1.575, -1.0, 0.3, 0.7]
RELEASE_CONFIG = [1.575, 0.893, -0.63, 1.28]
TRANSIT_CONFIG = [0.0, -0.47, 0.06, 1.97]
GRIPPER_CLOSE = [-0.01, 0.0]
GRIPPER_OPEN = [0.01, 0.0]

# Audio prerequisites
FORMAT = pyaudio.paInt16                                    # Suitable bit depth for processing audio files - matches audio bit depth used in CDs, without larger memory usage of 24/32 bit depths.
CHANNELS = 1                                                # Number of channels irrelevant in this application, simply set to mono.
RATE = 48000
FRAMES_PER_BUFFER = 1024

actionVerb = {
    "Get": ("Bring", "Find", "Fetch", "Give", "Hand", "Grab", "Pick", "Get", "bring", "find", "fetch", "give", "hand", "grab", "pick", "get")
}

hotword_prompt = {
    "Hello Speaker" : ("Hello Speaker", "Hello speaker", "hello speaker")
}

actionNoun_info = {                                         # Dictionary values may need to be altered into a list instead of a tuple to allow for modification in a dynamic map.
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
        self.listening_count = 0
        self.hotword_dict = hotword_prompt.get("Hello Speaker")
        try:
            self.doc = open("Summarisation_Layer.html", "x")
        except:
            pass

    def RecordAudio(self):
        rospy.loginfo("What would you like me to do?")


# Parameter setup for audio manipulation
        if self.hotword_detected:
            seconds = 5
            self.hotword_detected = False
            self.find_action = True
        else:
            seconds = 2
        p = pyaudio.PyAudio()

        stream = p.open(                                                # Initialisation of the parameters within the 'p' object of the PyAudio class.
            format = FORMAT,
            channels = CHANNELS,
            rate = RATE,
            input = True,
            frames_per_buffer=FRAMES_PER_BUFFER
        )

        with wave.open("Output.wav", "wb") as obj:                      # Create an object to save the .wav sample in a wave file. The file is set to 'write binary' for the allocatation of th frames list into the file.
            obj.setnchannels(CHANNELS)
            obj.setsampwidth(p.get_sample_size(FORMAT))
            obj.setframerate(RATE)

            for _ in range (0, int(RATE/FRAMES_PER_BUFFER*seconds)):    # Iterate through the frames list, filling up each element every second.
                data = stream.read(FRAMES_PER_BUFFER)                   # Number of frames per buffer stated on line 4 read per each iteration of the for loop. Returns a byte value.
                obj.writeframes(data)

            obj.close() 
        stream.stop_stream()
        stream.close()
        p.terminate()                                                   # Releases all the PortAudio resources.
        UserCommand.TranscribeAudio()

    def TranscribeAudio(self):
        self.listening_count += 1
        with open("Output.wav", "rb") as input_audio:
            self.transcription = openai.Audio.transcribe(
                file = input_audio,
                model = "whisper-1",
                response_format = "text",
                language = "en"
            )
            print(self.transcription)
        self.transcription = (self.transcription.replace(",",""))
        self.transcription = self.transcription.strip(" .")
        
        try:
            self.hotword = re.search("\AHello Speaker|\AHello speaker|\Ahello speaker", self.transcription)
            if self.hotword.group() in self.hotword_dict:
                ControlArm.move_gripper.go(GRIPPER_CLOSE, wait=True)
                ControlArm.move_gripper.go(GRIPPER_OPEN, wait=True)
                ControlArm.move_gripper.stop()
                ControlArm.move_gripper.clear_pose_targets()
                rospy.loginfo("LISTENING")
                self.hotword_detected = True
                self.RecordAudio()
        except:
            pass

        if self.find_action is True:
            self.find_action = False
            self.SummariseRequest(self.transcription)

    def SummariseRequest(self, request):
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "A user is asking a robot to pick up an object, the input text is a request from the user. Identify the main request that the user makes(for example, pick up the red block) and also rate how urgent the text is (1= not urgent, 10 = very urgent."},
                {"role": "user", "content": request}
            ]
        )
        summary = response["choices"][0]["message"]["content"]
        summary = summary.splitlines()
        split_summary = []
        for text in summary:
            if ": " in text:
                _,text = text.rsplit(": ", 1)
                split_summary.append(text.strip())
        
        split_summary[1] = int(split_summary[1])

        if split_summary[1] >= 8:
            self.reconfigure_params_fast()
        else:
            self.reconfigure_params_norm()

        self.final_request_str = split_summary[0]
        self.final_request_tupl = tuple(self.final_request_str.split())
        GoalAction.movetotarget(self.final_request_str, self.final_request_tupl)

    def reconfigure_params_fast(self):
        DWAPlanner_client = Client("/move_base/DWAPlannerROS")
        DWAparams = {
            "xy_goal_tolerance" : 0.075,
            "yaw_goal_tolerance" : 0.17,
            "path_distance_bias" : 64,
            "max_vel_trans" : 0.26,
        }
        inflation_layer_client = Client("/move_base/local_costmap/inflation_layer")
        inflation_params = {
            "cost_scaling_factor" : 4
        }
        DWAPlanner_client.update_configuration(DWAparams)
        inflation_layer_client.update_configuration(inflation_params)

    def reconfigure_params_norm(self):
        DWAPlanner_client = Client("/move_base/DWAPlannerROS")
        DWAparams = {
            "xy_goal_tolerance" : 0.05,
            "yaw_goal_tolerance" : 0.09,
            "path_distance_bias" : 32,
            "max_vel_trans" : 0.18,
        }
        inflation_layer_client = Client("/move_base/local_costmap/inflation_layer")
        inflation_params = {
            "cost_scaling_factor" : 4
        }
        DWAPlanner_client.update_configuration(DWAparams)
        inflation_layer_client.update_configuration(inflation_params)

class Navigate:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self.targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.wafflepose_locate = PoseStamped()
        self.amcl_poses = [0.0, 0.0]
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.home_iteration = False
        self.request_options = actionVerb.get("Get")
        self.running_status = False

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
            request.relative_entity_name = GLOBAL_FRAME

            # Get the model state
            response = self.targetobj_1(request)
            
            # Store the response (pose and orientation) in the dictionary
            responses[target] = {
                'pose': response.pose.position,
            }

        # Access the pose and orientation for each model
        redbox_pose = responses["red_box"]['pose']
        bluebox_pose = responses["blue_box"]['pose']
        greenbox_pose = responses["green_box"]['pose']

        actionNoun_info["red block"] = [
            round(redbox_pose.x, 6),
            round(redbox_pose.y, 6),
            round(redbox_pose.z, 6)
        ]

        actionNoun_info["blue block"] = [
            round(bluebox_pose.x, 6),
            round(bluebox_pose.y, 6),
            round(bluebox_pose.z, 6)
        ]

        actionNoun_info["green block"] = [
            round(greenbox_pose.x, 6),
            round(greenbox_pose.y, 6),
            round(greenbox_pose.z, 6)
        ]

    def movetotarget(self, result, result_period):
    # Directs the Waffle Pi to thetarget object pose
        # Acquire initial home pose position for object returns upon initialization
        if not self.home_iteration:
            self.home_pose = self.amcl_poses
            self.home_iteration = True
        for word in result_period:
            if word in self.request_options:
                try:
                    self.matched_object = re.search(r"(\bred|\bblue|\bgreen)\s(\bblock)", result)     # tweak pattern adherance to remove the whitespace at the end of the string
                except Exception as error:
                    rospy.logerr(f"Invalid arm location request: {error}")
                    break
                if self.matched_object:
                    self.matched_object = self.matched_object.group()
                                                        # Once initiated, the /amcl_pose topic publishes at a low frequency of approx. 0.66Hz. Unless there is a way to adjust this publishing frequency, a rudimentary 'buffer' must be implemented - a timer to allow the callback function to acquire the value.
                    if self.matched_object in actionNoun.keys():
                        self.orient_vals = self.base_orient()
                        self.wafflepose_locate.pose.position.x = actionNoun_info[self.matched_object][0] + self.orient_vals[0]
                        self.wafflepose_locate.pose.position.y = actionNoun_info[self.matched_object][1] + self.y_offset
                        self.wafflepose_locate.pose.orientation.z = self.orient_vals[1]
                        self.wafflepose_locate.pose.orientation.w = self.orient_vals[2]
                        self.wafflepose_locate.header.stamp = rospy.Time.now()
                        self.wafflepose_locate.header.frame_id = "map"

                        self.wafflepose = MoveBaseGoal()
                        self.wafflepose.target_pose = self.wafflepose_locate
                        base_client.send_goal(self.wafflepose)
                        self.running_status = True
                        print(self.wafflepose.target_pose.pose)
                        if base_client.wait_for_result():
                            base_client.cancel_goal()
                            self.wafflepose.target_pose = None                            
                        rospy.loginfo("TARGET OBJECT REACHED")
                        ControlArm.grabObject(self.matched_object)
            else:
                UserCommand.find_action = False

    def movetohome(self):
    # Directs the Waffle Pi to the home/initial pose.
        self.home_goal = MoveBaseGoal()
        self.home_target = PoseStamped()
        self.home_target.header.frame_id = "map"
        self.home_target.header.stamp = rospy.Time.now()
        self.home_target.pose.position.x = self.home_pose[0]
        self.home_target.pose.position.y = self.home_pose[1]
        self.home_target.pose.orientation.w = 1.0
        self.home_goal.target_pose = self.home_target
        base_client.send_goal_and_wait(self.home_goal)
        if base_client.wait_for_result():
            base_client.cancel_goal()
            self.wafflepose.target_pose = None
        rospy.loginfo("TASK COMPLETE")
        UserCommand.find_action = False

    def base_orient(self):
    # Orientates the Waffle Pi based on current positional error between the mobile base and the target.
    # item with respect to the x axis.

        self.x_error = (actionNoun_info[self.matched_object][0]-self.amcl_poses[0])
        self.y_error = (actionNoun_info[self.matched_object][1]-self.amcl_poses[1])
        self.y_offset = -0.22 if self.y_error > 0.0 else 0.22

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
        if not self.running_status:
            rospy.logdebug("AMCL ACTIVATED")
        # Calibrates the AMCL position of the Waffle Pi prior to initial movement.
            if self.home_iteration is False:
                amcl_pose = lidar_val.pose
                self.amcl_poses[0] = amcl_pose.pose.position.x
                self.amcl_poses[1] = amcl_pose.pose.position.y

            # rospy.logdebug("AMCL RUNNING")
            # Error checking is undertaken between the Pi pose and the assumed 
            # object pose within the global frame, setting an offset for the
            # target object pose along the y axis.
            # while True:
            #     if hasattr(self, "matched_object"):
            #         self.x_error = (actionNoun_info[GoalAction.matched_object][0]-self.amcl_poses[0])
            #         self.y_error = (actionNoun_info[GoalAction.matched_object][1]-self.amcl_poses[1])
            #         self.y_offset = -0.22 if self.y_error > 0.0 else 0.22
            #         break
            
class ControlTrajectory:
    def __init__(self):
        self.move_arm = MoveGroupCommander(GROUP_ARM)
        self.move_gripper = MoveGroupCommander(GROUP_GRIPPER)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.move_gripper.set_pose_reference_frame(GLOBAL_FRAME)
        self.move_gripper.set_goal_position_tolerance(GOAL_POSITION_TOLERANCE)
        self.move_arm.set_pose_reference_frame(NAVIGATION_FRAME)
        self.move_arm.set_goal_position_tolerance(GOAL_POSITION_TOLERANCE)
        self.move_arm.go(HOME_CONFIG)
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
        self.listener = tf.TransformListener()
        self.move_gripper.go(GRIPPER_OPEN, wait=True)
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        rospy.loginfo("Init Complete")

    def transform_to_base(self, pose):
        self.listener.waitForTransform("base_footprint", "map", rospy.Time(), rospy.Duration(4.0))
        pose_pretransform = PoseStamped()
        pose_pretransform.header.frame_id = "map"
        pose_pretransform.header.stamp = rospy.Time(0)
        pose_pretransform.pose = pose
        pose_base = self.listener.transformPose("base_footprint", pose_pretransform)
        print(pose_base)
        return pose_base

    def transform_to_odom(self, pose):
        self.listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(4.0))
        pose_pretransform = PoseStamped()
        pose_pretransform.header.frame_id = "base_footprint"
        pose_pretransform.header.stamp = rospy.Time(0)
        pose_pretransform.pose = pose
        pose_map = self.listener.transformPose("odom", pose_pretransform)
        return pose_map    

    def grabObject(self, item):
        waypoints = []
        state = PREP
        MAX_WAYPOINTS = 0
        GoalAction.setObjectInfo()

        if state == PREP:
            self.obj_pose = Pose()
            self.obj_pose.position.x = actionNoun_info[item][0]
            self.obj_pose.position.y = actionNoun_info[item][1]
            self.obj_pose.position.z = actionNoun_info[item][2]
            base_relative_objpose = self.transform_to_base(self.obj_pose)
            y_pose = base_relative_objpose.pose.position.y

            try:
                if y_pose > 0:
                    self.move_arm.go(LEFT_PICKUP, wait=True)
                    self.obj_pose.position.y += EE_TO_GRIPPERLINK_DIST
                elif y_pose < 0:
                    self.move_arm.go(RIGHT_PICKUP, wait=True)
                    self.obj_pose.position.y -= EE_TO_GRIPPERLINK_DIST
            except RuntimeError as object_pose_error:
                    rospy.logwarn(object_pose_error)
            else:
                state = ENTRY

        if state == ENTRY:
            init_pose = self.move_arm.get_current_pose().pose
            while True:
                init_pose.position.x = self.obj_pose.position.x
                init_pose.position.y = self.obj_pose.position.y
                waypoints.append(copy.deepcopy(init_pose))
                if len(waypoints) >= 50:
                    self.curr_entrypose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose.position
                    self.x_pose_error = self.obj_pose.position.x-self.curr_entrypose.x
                    self.y_pose_error = self.obj_pose.position.y-self.curr_entrypose.y
                    print(f"x error: {self.x_pose_error}")
                    print(f"y error: {self.y_pose_error}")
                    if (abs(self.x_pose_error) < 0.01) and (abs(self.y_pose_error) < 0.01):
                        state = ORIENT
                        self.move_arm.stop()
                        self.move_arm.clear_pose_targets()
                        waypoints.clear()
                        break
                (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, 0.0075)
                self.move_arm.execute(plan, wait=False)      
                rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break            

        if state == ORIENT:
            place_config = self.move_arm.get_current_joint_values()
            place_config[3] = 0.95
            self.move_arm.go(place_config, wait=True)
            state = DESCEND

        if state == DESCEND:
            descent_pose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose
            while True:
                descent_pose.position.z = self.obj_pose.position.z
                waypoints.append(copy.deepcopy(descent_pose))
                if len(waypoints) >= 10:
                    self.curr_descendpose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose.position.z
                    self.z_pose_error = self.obj_pose.position.z-self.curr_descendpose
                    print(self.z_pose_error)
                    if (abs(self.z_pose_error) < 0.022):
                        state = OBTAIN
                        self.move_arm.stop()
                        self.move_arm.clear_pose_targets()
                        waypoints.clear()
                        break
                (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, 0.01)
                self.move_arm.execute(plan, wait=False)      
                rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break    
            
        if state == OBTAIN:
            self.move_arm.stop()
            rospy.loginfo("CLOSING")
            self.move_gripper.go(GRIPPER_CLOSE, wait=True)
            rospy.sleep(1.5)
            state = ASCEND

        if state == ASCEND:
            self.move_arm.go(TRANSIT_CONFIG, wait=True)  
            rospy.sleep(2.0)
            print("OBJECT ACQUIRED")
            GoalAction.movetohome()
            state = RETURN 
        
        if state == RETURN:
            self.move_arm.go(RELEASE_CONFIG, wait=True)
            descent_placepose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose
            while True:
                MAX_WAYPOINTS += 1
                if MAX_WAYPOINTS % 5 == 0:
                    self.curr_returnpose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose.position.z
                    self.ee_curr_pose_z = self.obj_pose.position.z-self.curr_returnpose
                    if (abs(self.ee_curr_pose_z) < 0.035):
                        self.move_arm.stop()
                        self.move_arm.clear_pose_targets()
                        waypoints.clear()
                        state = RELEASE
                        break
                descent_placepose.position.z = 0.025
                waypoints.append(copy.deepcopy(descent_placepose))
                (plan, fraction) = self.move_arm.compute_cartesian_path(waypoints, 0.01)
                self.move_arm.execute(plan, wait=False)      
                rospy.sleep(0.05)
                if rospy.is_shutdown():
                    break
        
        if state == RELEASE:
            self.move_gripper.go(GRIPPER_OPEN, wait=True)
            rospy.sleep(1.0)
            self.move_arm.go(HOME_CONFIG, wait=True)
            rospy.sleep(1.0)
            self.move_arm.stop()
            self.move_gripper.stop()
            self.move_arm.clear_pose_targets()
            self.move_gripper.clear_pose_targets()
            state = None

class TaskThreader:
    def __init__(self):
        pass
    
    def start_SpeakBot(self):
        # Start Speakbot
        GoalAction.setObjectInfo()
        while not rospy.is_shutdown():
            UserCommand.RecordAudio()
            #UserCommand.SummariseRequest(UserCommand.request)
            #GoalAction.movetotarget(UserCommand.result, UserCommand.result_period)
            # GoalAction.movetotarget(GoalAction.result_string, GoalAction.result_tuple)
            # #rospy.sleep(1.0)            
            # #ControlArm.grabObject("green block")
            # GoalAction.running_status = False
            # UserCommand.find_action = False

    def start_ProximitySens(self):
        rospy.Subscriber("/scan", LaserScan, self.process_Scanner)
        rospy.spin()

    def process_Scanner(self, scan):
        lowest_val = sorted(scan.ranges)[:5]

        # The mean distance recorded is used instead of the minimum to mitigate anomalies in the sensor readings.
        lowest_val = round((stats.mean(lowest_val)), 3)

        if hasattr(GoalAction, "running_status"):
            if lowest_val < 0.5 and GoalAction.running_status is False:
                rospy.loginfo("OBJECT DETECTED WHILST STATIC")
            else:
                pass

if __name__ == '__main__': 
    rospy.init_node("speakbot")
    moveit_commander.roscpp_initialize(sys.argv)
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

    rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.04)
    rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.08)
    rospy.set_param("/move_base/DWAPlannerROS/path_distance_bias", 5.0)
    rospy.set_param("/move_base/DWAPlannerROS/max_vel_trans", 0.26)
    SpeakBot.start()
    Proximity_Sensor.setDaemon(True)
    Proximity_Sensor.start()