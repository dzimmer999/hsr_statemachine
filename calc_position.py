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


class CalculatePosition(smach.State):
    """Class for the CalculatePosition state of the statemachine"""

    def __init__(self):
        """Initited the CalculatePosition class"""

        smach.State.__init__(self,
                             outcomes=["calculated", "calc_error"],
                             input_keys=['marker_id'],
                             output_keys=['destination'])
        self.pose_pub = rospy.Publisher("/laser_2d_correct_pose", PoseWithCovarianceStamped, queue_size=10)
        self.transformer = tf.TransformListener(True, rospy.Duration(10))


    def execute(self, userdata):
        """Executes the state

        Args:
            userdata: Userdata of the statemachine. Used in this state are:
                userdata.marker_id (string): string of the id of the found marker
                userdata.destination (string): Name of the destination

        Returns:
            calculated (string): Managed to calculate the position and align RVIZ
            calc_error (string): Calculation error while trying to transform the tf
        """
        rospy.loginfo('Executing calculate position')
        id = userdata.marker_id
        userdata.destination = "kitchen"
        
        check = self.transform_marker(id)
        if check == 0:
            rospy.logerr("Couldn't calculate position.")
            return "calc_error"
        elif check == 1:
            return "calculated"
        else:
            print("How did you end here?")
        
    def transform_marker(self, id):
        """Transform the position from the robot to the position of the marker and then to the map origin
           (Parts: Source Elisabeth F, adjusted to be more stable)

        Args:
            id (string): id of the found marker

        Returns:
            double: 1 when the calculation was successfull
                    0 when an exception occured
        """

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
            pose_mid = self.transform_pose( f"ar_marker/{id}", "base_footprint",  pose_start)
            if pose_mid == 0:
                return 0
        except tf2_ros.TransformException:
            rospy.logwarn("Error while transforming to base")
            return 0
        else:
            try:
                pose_end = self.transform_pose("map", f"ar_marker/{id}fix",pose_mid) 
                if pose_mid == 0:
                    return 0
            except tf2_ros.TransformException:
                rospy.logwarn("Error while transforming from map")
                return 0

        pose_wcS = PoseWithCovarianceStamped()
        pose_wcS.header.frame_id = "map"
        pose_wcS.pose.pose = pose_end.pose
        pose_wcS.pose.covariance = np.zeros(36)

        self.pose_pub.publish(pose_wcS)
        rospy.loginfo("Published pose")
        return 1
    
    def transform_pose(self, target_frame, source_frame, pose_stamp):
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

        self.transformer.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
        try:
            target_pose = self.transformer.transformPose(target_frame, source_pose)
        except tf.Exception:
            rospy.logerr(f"Can't transform {target_frame} and {source_frame}")
            return 0
        else:
            rospy.loginfo(f"Can transform {target_frame} and {source_frame}")
            return target_pose

