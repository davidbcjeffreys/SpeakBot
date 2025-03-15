#!/usr/bin/env python3

from ultralytics import YOLO
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, GetLinkStateResponse
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf2_ros import BufferInterface
import numpy as np
import rospy
import tf
import cv2
import tf2_ros
import torch

obj_img_pose = [0.0, 0.0, 0.0]
depthY_val = 0
depthX_val = 0

class YOLO_ROS:
    def __init__(self): 
        self.model = YOLO()
        self.bridge = CvBridge()
        self.objpose = BufferInterface()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_processing)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_capture)
        self.detection_pub = rospy.Publisher("/yolo/detections", Image, queue_size=2)
        self.listener = tf.TransformListener()
        rospy.loginfo("YOLO ROS node initialized")
        self.row = 0

    def camera_processing(self, raw_image):
        # Convert ROS Image file to OpenCV Image file, specifically in blue-green-red format, since
        # OpenCV reads an image in this format.

        cv_image = self.bridge.imgmsg_to_cv2(raw_image,"bgr8")

        self.results = self.model(cv_image, max_det=200, device="cpu", conf=0.6)
        for result in self.results:
            boxes_data = result.boxes
        
        for object_num in range(len(boxes_data.xywhn)):

            global depthX_val, depthY_val

            boxes_objectX_tensor = torch.tensor(boxes_data.xywhn[object_num][0])
            boxes_objectY_tensor = torch.tensor(boxes_data.xywhn[object_num][1])
            boxes_depth_x_tensor = torch.tensor(boxes_data.xywh[object_num][0])
            boxes_depth_y_tensor = torch.tensor(boxes_data.xywh[object_num][1])

            objectX_val = boxes_objectX_tensor.item()
            objectY_val = boxes_objectY_tensor.item()
            depthX_val = boxes_depth_x_tensor.item()
            depthY_val = boxes_depth_y_tensor.item()

            approxDepth = self.depth_processing() / 8.5

            self.depthX = objectX_val * approxDepth / 2
            self.depthY = objectY_val * approxDepth / 2
            self.depthZ = approxDepth

            self.transform_to_global_frame()

        annotated_image = self.results[0].plot()
        ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        self.detection_pub.publish(ros_image)

    def depth_capture(self, depth_image):
        self.depth_capture_image = depth_image
    
    def depth_processing(self):
        global depthX_val, depthY_val

        cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_capture_image,"32FC1")
        cv_depth_image = np.nan_to_num(cv_depth_image, nan=0)
        normalized_depth_image = cv2.normalize(cv_depth_image, None, alpha=0, beta=255,norm_type=cv2.NORM_MINMAX)
        normalized_depth_image = np.uint8(normalized_depth_image)

        depthX_val = int(depthX_val)
        depthY_val = int(depthY_val)

        depth_value = normalized_depth_image[depthY_val, depthX_val]
        return depth_value
 

    def transform_to_global_frame(self):
        obj_frame_pose = PointStamped()
        obj_frame_pose.header.frame_id = "camera_rgb_optical_frame"
        obj_frame_pose.header.stamp = rospy.Time.now()
        obj_frame_pose.point.x = self.depthX
        obj_frame_pose.point.y = self.depthY
        obj_frame_pose.point.z = self.depthZ

        try:
            self.listener.waitForTransform("world", "camera_rgb_optical_frame", rospy.Time.now(), rospy.Duration(5.0))
            world_point = self.listener.transformPoint("world", obj_frame_pose)
            #rospy.loginfo(f"Tranformed point: {world_point.point}")
            print(f"Tranformed point: {world_point.point}")
        except LookupError as t:
            rospy.logwarn(f"Global transform error: {t}")


if __name__ == "__main__":
    rospy.init_node("ultralytics", anonymous=True)
    yolo_ros = YOLO_ROS()
    rospy.spin()