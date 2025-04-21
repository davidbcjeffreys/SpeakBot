#!/usr/bin/env/ python3

import rospy
import actionlib
import statistics
from dynamic_reconfigure.client import Client
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse

class Mover:
    def __init__(self):
        self.pose_locate = PoseStamped()
        self.waffle_curr_location = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        try:
            self.waffle_curr_location.wait_for_service(timeout=rospy.Duration(5.0))
            rospy.loginfo("Object localization service live")
        except ConnectionRefusedError as e:
            rospy.logerr(f"Service failed: {e}")

    def move_to_target(self):
        self.pose_locate.pose.position.x = 0.5
        self.pose_locate.pose.position.y = 1.5
        self.pose_locate.pose.position.z= 0.0
        self.pose_locate.pose.orientation.w = 1.0
        self.pose_locate.header.frame_id = "map"
        self.pose_locate.header.stamp = rospy.Time.now()

        self.pose = MoveBaseGoal()
        self.pose.target_pose = self.pose_locate
        self.start_time = rospy.Time.now().secs
        base_client.send_goal_and_wait(self.pose)
        self.end_time = rospy.Time.now().secs

    def show_duration(self):
        print(f"this is the trajectory duration: {self.end_time-self.start_time}")
    
    def show_pos_accuracy(self):
        self.x_acc = (self.response.pose.position.x / self.pose_locate.pose.position.x) * 100
        self.y_acc = (self.response.pose.position.y / self.pose_locate.pose.position.y) * 100
        self.w_acc = (self.response.pose.orientation.w / self.pose_locate.pose.orientation.w) * 100
        self.xy_vals = [self.x_acc, self.y_acc]
        self.ovr_xy_acc = statistics.mean(self.xy_vals)

        print(f"this is the xy positional accruacy: {self.ovr_xy_acc}")
        print(f"this is the w orientational accuracy: {self.w_acc}")

    def get_curr_pose(self):
        request = GetModelStateRequest()
        self.response = GetModelStateResponse()

        request.model_name = "robot"
        request.relative_entity_name = "map"

        self.response = self.waffle_curr_location(request)

if __name__ == "__main__":
    rospy.init_node("DWAPlanner_practice")
    base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
    try:
        base_client.wait_for_server(timeout=rospy.Duration(5.0))
        rospy.loginfo("Move base server connected successfully")
    except ConnectionRefusedError as e:
        rospy.logerr(f"Server failed: {e}")

    param_client = Client("/move_base/DWAPlannerROS")
    params = {
        "sim_time" : 3.0
    }
    param_client.update_configuration(params)

    # rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.05)
    # rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.17)
    # rospy.set_param("/move_base/DWAPlannerROS/path_distance_bias", 32.0)
    # rospy.set_param("/move_base/DWAPlannerROS/max_vel_trans", 0.26)
    # rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor", 1.0)
    # rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius", 1.0)

    # print(rospy.get_param("/move_base/DWAPlannerROS/xy_goal_tolerance"))
    # print(rospy.get_param("/move_base/DWAPlannerROS/yaw_goal_tolerance"))
    # print(rospy.get_param("/move_base/DWAPlannerROS/path_distance_bias"))
    # print(rospy.get_param("/move_base/DWAPlannerROS/max_vel_trans"))
    print(rospy.get_param("/move_base/DWAPlannerROS/sim_time"))
    # print(rospy.get_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor"))
    # print(rospy.get_param("/move_base/local_costmap/inflation_layer/inflation_radius"))

    practice_move = Mover()
    practice_move.move_to_target()
    practice_move.get_curr_pose()
    practice_move.show_duration()
    practice_move.show_pos_accuracy()
