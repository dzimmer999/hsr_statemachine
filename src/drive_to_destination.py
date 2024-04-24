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

class DriveToDestination(smach.State):
    """Class for the DriveToDestination state of the statemachine"""

    def __init__(self):
        """Initited the DriveToDestination class"""

        smach.State.__init__(self,
                             outcomes=["arrived_in_kitchen", "arrived_at_map", "retry", "error"],
                             input_keys=['error_count', 'destination'],
                             output_keys=['error_count', 'destination'])
        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')
        self.control_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.control_client.wait_for_server()
        self.max_tries = 3

    def execute(self, userdata):
        """Executes the state

        Args:
            userdata: Userdata of the statemachine. Used in this state are:
                userdata.error_count (double): counts how many times the robot failed to drive to the desination.
                                            When this exceeds self.max_tries the state switched to FindMarker.
                userdata.destination (string): Name of the desination (currently supported are "kitchen" and "map")

        Returns:
            arrived_in_kitchen (string): Robot arrived in the kitchen
            arrived_at_map (string): Robot arrived at the map origin
            retry (string): Robot failed to drive to destination and retries (userdata.error_count gets incremented)
            error (string): userdata.error_count exceeded self.max_tries. State switched to FindMarker
        """
        rospy.sleep(2)
        self.whole_body.move_to_go()
        try:   
            if userdata.destination == "kitchen":
                rospy.loginfo('Executing drive_to_kitchen')
                # TODO: Fix this, robot sometimes stops moving
                # Idea: Split path into three thirds
                # Problem: What if robot already close to kitchen? Calculate distance? -> annoying
                self.omni_base.go_abs(-0.2, -2.36, math.pi, 1000)
            elif userdata.destination == "map":
                rospy.loginfo('Executing drive_to_map')
                self.omni_base.go_abs(0, 0, 0, 1000)
            else:
                rospy.logerr(f"Please specify destination {userdata.destination}.")
                return "error"
        except hsrb_interface.exceptions.MobileBaseError as e:
            if userdata.error_count >= self.max_tries:
                rospy.logerr("Failed to reach goal (drive_to_kitchen.py).")
                print(e)
                userdata.error_count = 0
                return "error"
            else:
                userdata.error_count += 1
                rospy.logerr(f"Failed to reach goal (Try {userdata.error_count}/{self.max_tries}) (drive_to_kitchen.py).")
                return "retry"
        else:
            if userdata.destination == "kitchen":
                return "arrived_in_kitchen"
            elif userdata.destination == "map":
                return "arrived_at_map"
            else:
                rospy.logerr(f"Please specify destination {userdata.destination}.")
                return "error"


        