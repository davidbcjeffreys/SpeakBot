#!/usr/bin/env python3
# 
# GetInstruction class
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1

import rospy
import pyaudio
import wave
import openai
import re
from dynamic_reconfigure.client import Client

# Audio prerequisites
FORMAT = pyaudio.paInt16            # Suitable bit depth for this use case
CHANNELS = 1                        # Number of channels irrelevant in this application, simply set to mono.
RATE = 48000
FRAMES_PER_BUFFER = 1024

# move_group group's and global frame declarations
GLOBAL_FRAME = "map"
NAVIGATION_FRAME = "odom"
GROUP_ARM = "arm"
GROUP_GRIPPER = "gripper"

# Joint configurations
GRIPPER_CLOSE = [-0.01, 0.0]
GRIPPER_OPEN = [0.01, 0.0]

hotword_prompt = {
    "Hello Speaker" : ("Hello Speaker", "Hello speaker", "hello speaker")
}

class GetInstruction:
    """
    Class for obtaining vocal input from the user and processing it. The audio is sampled
    on the default system microphone before being fed to the OpenAI whisper-1 ASR system.
    The resulting transcription is scrutinised in OpenAI's GPT-3.5-Turbo ChatCompletion
    function allowing the simple user request and user request urgency (returned as an 
    int: 1 = minimal urgency, 10 = maximum urgency) to be obtained.
    """
    def __init__(self):
        """Set a sane state for the hotword detection"""
        self.hotword_detected, self.find_action = False, False
        self.hotword_dict = hotword_prompt.get("Hello Speaker")
    
    def getObjects(self, manipulator_obj, navigation_obj):
        self.ControlTrajectory = manipulator_obj
        self.Navigate = navigation_obj

    def RecordAudio(self):
        """Record the audio from the default microphone in WAV format for quality retention. The audio is sampled every 2 seconds without hotword detection and every 5 seconds once the hotword is detected."""
        rospy.loginfo("What would you like me to do?")        
        if self.hotword_detected:
            seconds = 5
            self.hotword_detected, self.find_action = False, True
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
        self.TranscribeAudio()
    
    def TranscribeAudio(self):
        """Transcribe the audio file from RecordAudio using Whisper. RegEx utilised to identify the presence of the hotword in the input transcription."""
        with open("Output.wav", "rb") as input_audio:
            self.transcription = openai.Audio.transcribe(
                file = input_audio,
                model = "whisper-1",
                response_format = "text",
                language = "en"
            )
        self.transcription = (self.transcription.replace(",",""))
        self.transcription = self.transcription.strip(" .")
        
        # Identify hotword presence
        try:
            self.hotword = re.search("\AHello Speaker|\AHello speaker|\Ahello speaker", self.transcription)
            if self.hotword.group() in self.hotword_dict:
                self.ControlTrajectory.move_gripper.go(GRIPPER_CLOSE, wait=True)
                self.ControlTrajectory.move_gripper.go(GRIPPER_OPEN, wait=True)
                self.ControlTrajectory.move_gripper.stop()
                self.ControlTrajectory.move_gripper.clear_pose_targets()
                rospy.loginfo("Listening...")
                self.hotword_detected = True
                self.RecordAudio()
        except:
            pass

        if self.find_action is True:
            self.find_action = False
            self.SummariseRequest(self.transcription)
    
    def SummariseRequest(self, request):
        """
        Summarization layer containing the simple user request and urgency weighting for the request from the user.
        
        Parameters
        ----------
        request : string

        Example
        -------
        "I really need the green block, could you get it for me as soon as possible please?"

        """
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

        self.DWAPlanner_client.update_configuration(self.DWAparams)
        self.final_request_str = split_summary[0]
        self.final_request_tupl = tuple(self.final_request_str.split())
        self.Navigate.movetotarget(self.final_request_str, self.final_request_tupl)

    """Enables dynamic reconfiguration of the DWA ROS Planner parameters based on user request urgency."""
    def reconfigure_params_fast(self):
        """ Faster path bias"""
        self.DWAPlanner_client = Client("/move_base/DWAPlannerROS")
        self.DWAparams = {
            "xy_goal_tolerance" : 0.075,
            "yaw_goal_tolerance" : 0.17,
            "path_distance_bias" : 64,
            "max_vel_trans" : 0.26,
        }

    def reconfigure_params_norm(self):
        """ Normal path bias"""
        self.DWAPlanner_client = Client("/move_base/DWAPlannerROS")
        self.DWAparams = {
            "xy_goal_tolerance" : 0.05,
            "yaw_goal_tolerance" : 0.09,
            "path_distance_bias" : 32,
            "max_vel_trans" : 0.18,
        }
