#!/usr/bin/env python3
#
# TaskThreader class
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1

import rospy
import statistics as stats
from sensor_msgs.msg import LaserScan

class TaskThreader:
    """
    Handles concurrent operation of the proximity sensor and the main SpeakBot thread
    through pre-emptive, OS-controlled multi-threading. Readings from the proximity
    sensor are nullified once the over-arching process receives a flag stating that
    SpeakBot is running: the proximity sensor only alerts the user whilst the robot is
    stationary.
    """
    def __init__(self):
        pass

    def getObjects(self, navigation_obj, instruction_obj):
        self.Navigate = navigation_obj
        self.GetInstruction = instruction_obj

    def start_SpeakBot(self):
        self.Navigate.setObjectInfo()
        while not rospy.is_shutdown():
            self.GetInstruction.RecordAudio()

    def start_ProximitySens(self):
        rospy.Subscriber("/scan", LaserScan, self.process_Scanner)
        rospy.spin()

    def process_Scanner(self, scan):
        lowest_val = sorted(scan.ranges)[:5]

        # The mean distance recorded is used instead of the minimum to mitigate anomalies in the sensor readings.
        lowest_val = round((stats.mean(lowest_val)), 3)

        if hasattr(self.Navigate, "running_status"):
            if lowest_val < 0.5 and self.Navigate.running_status is False:
                rospy.loginfo("OBJECT DETECTED WHILST STATIC")
            else:
                pass