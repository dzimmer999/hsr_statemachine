#!/usr/bin/python3

import rospy
import smach
import numpy as np
import hsrb_interface
import actionlib
import tf.transformations
import math
import tf
import tf2_ros
import yaml

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tmc_vision_msgs.msg import MarkerArray

def transform_pose(target_frame, source_frame, pose_stamp): 
    """Transforming the given pose (pose_stamp) from the tf "source_frame" to the tf "target_frame"
        (Source Elisabeth F)

    Args:
        target_frame (string): Name of the target tf
        source_frame (string): Name of the source tf
        pose_stamp (geometry_msgs.PoseStamped): Containing the header and pose (position and orientation)

    Returns:
        0 (double): When the pose can't be transformed
        target_pose (PoseStamped): Transformed pose when the transformation was successfull
    """
    target_pose = PoseStamped()
    source_pose = PoseStamped()
    source_pose.header.frame_id = source_frame
    source_pose.header.stamp = rospy.Time(0)
    source_pose.pose = pose_stamp.pose

    listener = tf.TransformListener(True, rospy.Duration(10))

    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    print("Transforming")
    try:
        target_pose = listener.transformPose(target_frame, source_pose)
    except tf.Exception:
        rospy.logerr("Transform failure")
        rospy.loginfo(f"Can't transform {target_frame} and {source_frame}")
        return 0
    else:
        rospy.loginfo(f"Can transform {target_frame} and {source_frame}")
        return target_pose

if __name__ == "__main__":
    """Calculated and prints the transformation from the given marker to the map origin.
       The result is used in tf_broadcaster to broadast the fixed tf.
    """

    rospy.init_node("tf_calc")
    robot = hsrb_interface.Robot()
    body = robot.get('whole_body')
    body.move_to_joint_positions({'head_tilt_joint': -0.6})
    body.move_to_joint_positions({'arm_flex_joint':0})
    body.move_to_joint_positions({'arm_roll_joint':0.8})
    id = 15

    # Pose in map
    pose_end = PoseStamped()
    # Pose in marker
    pose_mid = PoseStamped()
    # Pose in robot (origin)
    pose_start = PoseStamped()
    pose_start.pose.position.x=0
    pose_start.pose.position.y=0
    pose_start.pose.position.z=0
    pose_start.pose.orientation.x=0
    pose_start.pose.orientation.y=0
    pose_start.pose.orientation.z=0
    pose_start.pose.orientation.w=0
    time = rospy.Time.to_sec(rospy.Time.now())

    try:
        pose_mid = transform_pose("map", f"ar_marker/{id}",  pose_start)
        if pose_mid == 0:
            print("error")
    except tf2_ros.TransformException:
        rospy.logwarn("Error while transforming")
        print("error")

    print(pose_mid.pose)
