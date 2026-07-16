#!/usr/bin/env python3
"""ROS1 ground_truth_odom: Gazebo link_states -> odom->base_footprint TF (tf2)
Implements deduplication: only publishes TF when pose changes beyond threshold,
preventing TF_REPEATED_DATA errors that break gmapping's TF lookups."""

import rospy
import tf2_ros
import math
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped


class GroundTruthOdom:
    def __init__(self):
        rospy.init_node("ground_truth_odom")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.cb)
        self.last_pose = None
        rospy.loginfo("ground_truth_odom started (tf2, with dedup)")

    def cb(self, msg):
        try:
            idx = msg.name.index("tron2_robot::base_Link")
        except ValueError:
            return
        pose = msg.pose[idx]
        x, y = pose.position.x, pose.position.y
        qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # dedup: skip if pose unchanged beyond threshold (1e-4 m / 1e-4 rad)
        if self.last_pose is not None:
            lx, ly, lyaw = self.last_pose
            if (abs(x - lx) < 1e-4 and abs(y - ly) < 1e-4
                    and abs(yaw - lyaw) < 1e-4):
                return
        self.last_pose = (x, y, yaw)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.br.sendTransform(t)


if __name__ == "__main__":
    node = GroundTruthOdom()
    rospy.spin()
