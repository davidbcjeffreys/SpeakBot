#!/usr/bin/env python3
#
# SpeakBot.py
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1
#
# Overarching script for SpeakBot initialization. Multi-threaded proximity sensing and primary SpeakBot trigger are declared within this script.

import rospy
import moveit_commander
import threading
import sys
from TaskThreader import TaskThreader
from NavigateTurtleBot import Navigate
from GetInstruction import GetInstruction
from ControlTrajectory import ControlTrajectory

if __name__ == '__main__':
    """Initialise Speakbot nodes & declare module objects"""
    rospy.init_node("speakbot")
    moveit_commander.roscpp_initialize(sys.argv)
    setRequest = GetInstruction()
    directTurtleBot = Navigate()
    driveManipulator = ControlTrajectory()
    SpeakBot_Thread = TaskThreader()

    """Derive objects for cross-module function utilisation"""
    setRequest.getObjects(navigation_obj=directTurtleBot, manipulator_obj=driveManipulator)
    directTurtleBot.getObjects(manipulator_obj=driveManipulator,instruction_obj=setRequest)
    driveManipulator.getObjects(navigation_obj=directTurtleBot)
    SpeakBot_Thread.getObjects(instruction_obj=setRequest, navigation_obj=directTurtleBot)

    """Inititate execution and proximity sensing threads"""
    SpeakBot = threading.Thread(target=SpeakBot_Thread.start_SpeakBot)
    Proximity_Sensor = threading.Thread(target=SpeakBot_Thread.start_ProximitySens)
    SpeakBot.start()
    Proximity_Sensor.setDaemon(True)
    Proximity_Sensor.start()