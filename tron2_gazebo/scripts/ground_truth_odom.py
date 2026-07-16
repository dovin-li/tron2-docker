#!/usr/bin/env python3
"""ROS1 ground_truth_odom: Gazebo link_states -> odom->base_footprint TF (tf2)"""
import rospy, tf2_ros, math
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped

class GroundTruthOdom:
    def __init__(self):
        rospy.init_node("ground_truth_odom")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.cb)
        rospy.loginfo("ground_truth_odom started (tf2)")

    def cb(self, msg):
        try:
            idx = msg.name.index("tron2_robot::base_Link")
        except ValueError:
            return
        p = msg.pose[idx]
        qw,qx,qy,qz = p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z
        yaw = math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw/2)
        t.transform.rotation.w = math.cos(yaw/2)
        self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("ground_truth_odom")
    rospy.spin()
