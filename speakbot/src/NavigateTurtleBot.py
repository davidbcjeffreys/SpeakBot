#!/usr/bin/env python3
#
# Navigate class
#
# Author:   David Jeffreys
# Date:     3/5/2025
# Version:  1.1

import rospy
import re
import actionlib
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

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

# actionVerb dictionary containing the potential alternative instructive verbs from the user (upper and lowercase)
actionVerb = {
    "Get": ("Bring", "Find", "Fetch", "Give", "Hand", "Grab", "Pick", "Get", "bring", "find", "fetch", "give", "hand", "grab", "pick", "get")
}

actionNoun = {
    "red block"     : ["red box", "red block", "red object"],
    "blue block"     : ["blue box", "blue block", "blue object"],
    "green block"     : ["green box", "green block", "green object"]
}

class Navigate:
    """
    This class uses the input request to direct the Turtlebot WafflePi towards a target pose which
    is offset from the actual target object pose. To determine this, the difference between the current
    pose from the AMCL and the target pose is calculated.
    """
    def __init__(self):
        """Ensure model state broadcasting service is live from Gazebo and prepare AMCL pose buffer."""
        rospy.wait_for_service("/gazebo/get_model_state")
        self.wafflepose_locate = PoseStamped()
        self.amcl_poses = [0.0, 0.0]
        self.y_offset = 0.0
        self.home_iteration, self.running_status = False, False
        self.request_options = actionVerb.get("Get")
        self.targetobj_1 = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.actionNoun_info = {                                         # Dictionary values may need to be altered into a list instead of a tuple to allow for modification in a dynamic map.
            "red block"          : [], 
            "green block"        : [],
            "blue block"         : [],
            }

        try:
            self.base_client.wait_for_server(timeout=rospy.Duration(5.0))
            rospy.loginfo("Move base server connected successfully")
        except ConnectionRefusedError as e:
            rospy.logerr(f"Server failed: {e}")

        try:
            self.targetobj_1.wait_for_service(timeout=rospy.Duration(5.0))
            rospy.loginfo("Object localization service live")
        except ConnectionRefusedError as e:
            rospy.logerr(f"Service failed: {e}")

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_AMCL_pose, queue_size=1) 

    def getObjects(self, instruction_obj, manipulator_obj):
        self.ControlTrajectory = manipulator_obj
        self.GetInstruction = instruction_obj
    
    def setObjectInfo(self):
        """
        Obtains the pose of all the known/required objects in the domain via Gazebo service.
        Notably, this means that all the positions must be obtained before further computing
        cycles can be undertaken - a timely process, undertaken each time a new request is made.
        Future implementation may focus on only updating the information for the exact object
        requested. In practice, regular mapping of the domain (and object pose recording) would
        address this potential bottleneck.
        """

        target_objects = ["red_box", "blue_box", "green_box"]
        responses = {}

        # Acquiring information for each object within the dataset.
        for target in target_objects:
            request = GetModelStateRequest()
            response = GetModelStateResponse()
            
            request.model_name = target
            request.relative_entity_name = GLOBAL_FRAME

            # Get the object state
            response = self.targetobj_1(request)
            
            # Store the position in the dictionary
            responses[target] = {
                'pose': response.pose.position,
            }

        # Access the pose and orientation for each object
        redbox_pose = responses["red_box"]['pose']
        bluebox_pose = responses["blue_box"]['pose']
        greenbox_pose = responses["green_box"]['pose']

        self.actionNoun_info["red block"] = [
            round(redbox_pose.x, 6),
            round(redbox_pose.y, 6),
            round(redbox_pose.z, 6)
        ]

        self.actionNoun_info["blue block"] = [
            round(bluebox_pose.x, 6),
            round(bluebox_pose.y, 6),
            round(bluebox_pose.z, 6)
        ]

        self.actionNoun_info["green block"] = [
            round(greenbox_pose.x, 6),
            round(greenbox_pose.y, 6),
            round(greenbox_pose.z, 6)
        ]

    def movetotarget(self, result, result_period):
        """
        Directs the Waffle Pi to the target object pose, factoring in the offset.
        
        Parameters
        ----------
        result : string
        result_period : tuple

        Example
        -------

        >>> result = "I really need the green block, could you get it for me as soon as possible please?"
        >>> result_period = ("I" , "really", "need"....)
        """
        # Acquire initial home pose position for object returns upon initialization
        if not self.home_iteration:
            self.home_pose = self.amcl_poses
            self.home_iteration = True
        
        # Iterates through string request tuple to ensure that user's request contains target objects present in the current domain
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
                        self.wafflepose_locate.pose.position.x = self.actionNoun_info[self.matched_object][0] + self.orient_vals[0]
                        self.wafflepose_locate.pose.position.y = self.actionNoun_info[self.matched_object][1] + self.y_offset
                        self.wafflepose_locate.pose.orientation.z = self.orient_vals[1]
                        self.wafflepose_locate.pose.orientation.w = self.orient_vals[2]
                        self.wafflepose_locate.header.stamp = rospy.Time.now()
                        self.wafflepose_locate.header.frame_id = "map"
                        self.wafflepose = MoveBaseGoal()
                        self.wafflepose.target_pose = self.wafflepose_locate
                        self.base_client.send_goal(self.wafflepose)
                        self.running_status = True
                        if self.base_client.wait_for_result():
                            self.base_client.cancel_goal()
                            self.wafflepose.target_pose = None                            
                        rospy.loginfo("TARGET OBJECT REACHED")
                        self.ControlTrajectory.grabObject(self.matched_object)
            else:
                self.GetInstruction.find_action = False
    
    def movetohome(self):
        """Directs the Waffle Pi to the home/initial pose."""
        self.home_goal = MoveBaseGoal()
        self.home_target = PoseStamped()
        self.home_target.header.frame_id = "map"
        self.home_target.header.stamp = rospy.Time.now()
        self.home_target.pose.position.x = self.home_pose[0]
        self.home_target.pose.position.y = self.home_pose[1]
        self.home_target.pose.orientation.w = 1.0
        self.home_goal.target_pose = self.home_target
        self.base_client.send_goal_and_wait(self.home_goal)
        if self.base_client.wait_for_result():
            self.base_client.cancel_goal()
            self.wafflepose.target_pose = None
        rospy.loginfo("TASK COMPLETE")
        self.GetInstruction.find_action = False
    
    def base_orient(self):
        """Orientates the Waffle Pi based on current positional error between the mobile base and the target item with respect to the x axis."""
        self.x_error = (self.actionNoun_info[self.matched_object][0]-self.amcl_poses[0])
        self.y_error = (self.actionNoun_info[self.matched_object][1]-self.amcl_poses[1])
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
        """
        Acquires the Adaptive Monte Carlo estimated pose
        
        Parameters
        ----------
        lidar_val : geometry_msgs.msg/PoseWithCovarianceStamped
        """
        if not self.running_status:
            rospy.loginfo("AMCL Running...")
        # Calibrates the AMCL position of the Waffle Pi prior to initial movement.
            if self.home_iteration is False:
                amcl_pose = lidar_val.pose
                self.amcl_poses[0] = amcl_pose.pose.position.x
                self.amcl_poses[1] = amcl_pose.pose.position.y