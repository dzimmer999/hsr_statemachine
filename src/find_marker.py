#!/usr/bin/python3
# Parts are: Copyright (C) 2016 Toyota Motor Corporation
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

class FindMarker(smach.State):
    """Class for the FindMarker state of the statemachine"""

    def __init__(self):
        """Initited the FindMarker class"""

        smach.State.__init__(self,
                             outcomes=["found", "not_found"],
                             output_keys=['marker_id'])
        
        self.start = False
        self.marker_sub = rospy.Subscriber("/ar_marker", MarkerArray, self.marker_cb)
        self.marker_id = None

        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')

        self.head_pan_low = -1.745
        self.head_pan_high = 1.745
        self.pan_steps = 3
        self.head_pan_step = (self.head_pan_high - self.head_pan_low)/self.pan_steps
        self.direction = 1

        self.head_tilt_low = 0.0
        self.head_tilt_high = 0.0
        self.tilt_steps = 1
        self.head_tilt_step = (self.head_tilt_high - self.head_tilt_low)/self.tilt_steps

    def execute(self, userdata):
        """Executes the state

        Args:
            userdata: Userdata of the statemachine. Used in this state are:
                userdata.marker_id (string): id of the marker

        Returns:
            found (string): Marker was found
            not_found (string): Marker was not found
        """
        rospy.loginfo('Executing state find_marker')
        
        self.whole_body.move_to_go()
        # self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.2})
        # self.whole_body.move_to_joint_positions({'arm_lift_joint': 0})
        # self.whole_body.move_to_joint_positions({'arm_roll_joint': 0.5}) 
        self.whole_body.move_to_joint_positions({'head_pan_joint': self.head_pan_low})
        self.whole_body.move_to_joint_positions({'head_tilt_joint': self.head_tilt_high})
       
        head_pan = self.head_pan_low
        head_tilt = self.head_tilt_high

        # self.start = True

        while not rospy.is_shutdown() and self.marker_id is None:

            try:
                if head_pan <= self.head_pan_low + 0.1:
                    self.direction = 1
                    head_tilt -= self.head_tilt_step
                    self.whole_body.move_to_joint_positions({'head_tilt_joint': head_tilt})
                elif head_pan >= self.head_pan_high - 0.1:
                    self.direction = -1
                    head_tilt -= self.head_tilt_step
                    self.whole_body.move_to_joint_positions({'head_tilt_joint': head_tilt})

                if head_tilt <= self.head_tilt_low -0.1:
                    rospy.loginfo("Couldn't find a marker.")
                    return "not_found"

                head_pan += self.head_pan_step * self.direction
                self.whole_body.move_to_joint_positions({'head_pan_joint': head_pan})
                rospy.sleep(0.3)

            except hsrb_interface.exceptions.MotionPlanningError as e:
                rospy.logerr("Couldn't change joint position.")
                return "not_found"

        userdata.marker_id = self.marker_id
        return "found"

    def marker_cb(self, msg):
        """Callback function of the /ar_marker subscriber

        Args:
            msg (tmc_vision_msgs.MarkerArray): Message containing all found markers
        """

        #if self.start is False:
        #    return

        if len(msg.frame_ids) > 0:
            # Found a marker, taking the first marker
            id = msg.frame_ids[0].split("/")[1]

            # TODO: Add all markers
            if id == "17" or id == "20":
                self.marker_id = id

