#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class FixedGlobalBroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage, queue_size=1)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
           rate.sleep()

           t = geometry_msgs.msg.TransformStamped()
           t.header.frame_id = "world"
           t.header.stamp = rospy.Time.now()
           t.child_frame_id = "map"
           t.transform.translation.x = 0.0
           t.transform.translation.y = 0.0
           t.transform.translation.z = 0.0
           t.transform.rotation.x = 0.0
           t.transform.rotation.y = 0.0
           t.transform.rotation.z = 0.0
           t.transform.rotation.w = 1.0

           tfm = tf2_msgs.msg.TFMessage([t])
           self.pub_tf.publish(tfm)

if __name__ == "__main__":
    rospy.init_node("fixed_global_broadcaster")
    tfb = FixedGlobalBroadcaster()

    rospy.spin()