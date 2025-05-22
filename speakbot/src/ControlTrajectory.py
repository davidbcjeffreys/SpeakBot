#!/usr/bin/env python3
#
# ControlTrajectory class
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1

import tf
import rospy
import copy
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped

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

# Joint configurations
HOME_CONFIG = [0.0, -1.0, 0.3, 0.7]
RIGHT_PICKUP = [-1.575, -1.0, 0.3, 0.7]
LEFT_PICKUP = [1.575, -1.0, 0.3, 0.7]
RELEASE_CONFIG = [1.575, 0.893, -0.63, 1.28]
TRANSIT_CONFIG = [0.0, -0.47, 0.06, 1.97]
GRIPPER_CLOSE = [-0.01, 0.0]
GRIPPER_OPEN = [0.01, 0.0]

# Arm states
PREP = 0
ENTRY = 1
ORIENT = 2
DESCEND = 3
OBTAIN = 4
ASCEND = 5
RETURN = 6
RELEASE = 7

class ControlTrajectory:
    """
    Controls trajectory planning and control of the attached 5DOF arm
    """
    def __init__(self):
        """Initialises MoveGroupCommander for the robotic arm, setting the arm in a sane (home) config."""
        self.move_arm = MoveGroupCommander(GROUP_ARM)
        self.move_gripper = MoveGroupCommander(GROUP_GRIPPER)
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
        rospy.loginfo("Manipulator Init Complete")
    
    def getObjects(self, navigation_obj):
        self.Navigate = navigation_obj

    def transform_to_base(self, pose):
        """
        Transforms a pose in the global frame to the mobile robot base frame.
        
        Parameters
        ----------
        pose : geometry_msgs.msg/Pose
        """
        self.listener.waitForTransform("base_footprint", "map", rospy.Time(), rospy.Duration(4.0))
        pose_pretransform = PoseStamped()
        pose_pretransform.header.frame_id = "map"
        pose_pretransform.header.stamp = rospy.Time(0)
        pose_pretransform.pose = pose
        pose_base = self.listener.transformPose("base_footprint", pose_pretransform)
        return pose_base

    def transform_to_odom(self, pose):
        """
        Transforms a pose in the mobile robot base frame to the odometer frame.
        
        Parameters
        ----------
        pose : geometry_msgs.msg/Pose
        """
        self.listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(4.0))
        pose_pretransform = PoseStamped()
        pose_pretransform.header.frame_id = "base_footprint"
        pose_pretransform.header.stamp = rospy.Time(0)
        pose_pretransform.pose = pose
        pose_map = self.listener.transformPose("odom", pose_pretransform)
        return pose_map
    
    def grabObject(self, item):
        rospy.logwarn("CODE IS AT THIS POINT IN CONTROL")
        """
        Obtains the object. Implemented as an 8 state FSM, grabObject integrates a cartesian
        path computer to derive the IK for the arm. RRT Connect is utilised in the testing of
        this class, though this is the default recommendation: it is not a rigid choice. It is 
        advised to experiment with other planning algorithms.

        Parameters
        ----------
        item : string

        Examples
        --------
        >>> "red box", "green box"
        """
        waypoints = []
        state = PREP
        MAX_WAYPOINTS = 0
        self.Navigate.setObjectInfo()

        # Acquires the object pose data for the target object, then uses this to determine a cartesian offset.
        if state == PREP:
            self.obj_pose = Pose()
            self.obj_pose.position.x = self.Navigate.actionNoun_info[item][0]
            self.obj_pose.position.y = self.Navigate.actionNoun_info[item][1]
            self.obj_pose.position.z = self.Navigate.actionNoun_info[item][2]
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
        
        # Plans, processes and updates a path for the robotic arm in the planar (XY) plane - Z is retained at the same value.
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
        
        # Orients the end effector towards the ground
        if state == ORIENT:
            place_config = self.move_arm.get_current_joint_values()
            place_config[3] = 0.95
            self.move_arm.go(place_config, wait=True)
            state = DESCEND

        # Plans, processes and updates a path for the robotic arm on the Z axis - X/Y movement is not administered here.
        if state == DESCEND:
            descent_pose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose
            while True:
                descent_pose.position.z = self.obj_pose.position.z
                waypoints.append(copy.deepcopy(descent_pose))
                if len(waypoints) >= 10:
                    self.curr_descendpose = self.transform_to_odom(self.move_arm.get_current_pose().pose).pose.position.z
                    self.z_pose_error = self.obj_pose.position.z-self.curr_descendpose
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
        
        # Close the bi-directional grippers to obtain the object
        if state == OBTAIN:
            self.move_arm.stop()
            rospy.loginfo("Closing...")
            self.move_gripper.go(GRIPPER_CLOSE, wait=True)
            rospy.sleep(1.5)
            state = ASCEND

        # Moves the robotic arm into a sane configuration for object transit
        if state == ASCEND:
            self.move_arm.go(TRANSIT_CONFIG, wait=True)  
            rospy.sleep(2.0)
            print("Object acquired!...")
            self.Navigate.movetohome()
            state = RETURN 
        
        # Plans a cartesian path for the robotic arm to drop the object
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
        
        # Opens the gripper to release the object
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
