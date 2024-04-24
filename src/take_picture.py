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
import datetime
import os

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tmc_vision_msgs.msg import MarkerArray

from cv_bridge import CvBridge, CvBridgeError
cv_bridge = CvBridge()
import cv2
from PIL import Image as PIL_Image
from sensor_msgs.msg import Image

class TakePicture(smach.State):
    """Class for the TakePicture state of the statemachine"""

    def __init__(self):
        """Initited the TakePicture class"""

        smach.State.__init__(self, outcomes=["done", "error"], output_keys=["destination"])
        
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.image_cb)
        self.image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "images", "og_img")
        os.chdir(self.image_path)

        self.marker_sub = rospy.Subscriber("/ar_marker", MarkerArray, self.marker_cb)
        self.marker_id = None

        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')

        current_datetime = datetime.datetime.now()
        self.formatted_datetime = current_datetime.strftime("%Y%m%d_%H%M")


    def execute(self, userdata):
        """Executes the state

        Args:
            userdata: Userdata of the statemachine. Used in this state are:
                userdata.destination (string): String of the destination

        Returns:
            done (string): Pictures got taken and saved
            error (string): Error while taking or saving the images
        """

        rospy.loginfo('Executing state take picture')
        userdata.destination = "map"

        rospy.loginfo("Saving images")

        # POSITION 1
        self.take_picture(-0.3, -2.3, math.pi, 1)

        # POSITION 2
        self.take_picture(0.4, -2.3, math.pi, 2)

        # POSITION 3
        self.take_picture(1.1, -2.3, math.pi, 3)

        # POSITION 4
        self.omni_base.go_abs(0.2, -3.5, math.pi, 1000)
        self.picture_pos()
        self.whole_body.move_to_joint_positions({'head_pan_joint': -math.pi/2})
        self.whole_body.move_to_joint_positions({'head_tilt_joint': -0.1})

        result = cv2.imwrite(f"kitchen_image_{self.formatted_datetime}_4_.png", self.cv2_img)
        if not result:
            rospy.logerr("Error while saving the image.")
            return "error"

        return "done"


    def marker_cb(self, msg):
        """Callback function of the /ar_marker subscriber

        Args:
            msg (tmc_vision_msgs.MarkerArray): Message containing all found markers
        """
        # Currently unused
        if len(msg.frame_ids) > 0:
            # Found a marker, taking the first marker
            id = msg.frame_ids[0].split("/")[1]
            self.marker_id = id
        
    def image_cb(self, msg):
        """Callback function of the /ar_marker subscriber

        Args:
            msg (sensor_msgs.Image): Image taken by the head camera of the robot
        """

        try:
            self.cv2_img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            rospy.logerr("Error while transforming image to cv2")

    def picture_pos(self):
        self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.5})
        self.whole_body.move_to_joint_positions({'arm_roll_joint': math.pi/2})
        self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.69})
        self.whole_body.move_to_joint_positions({'head_tilt_joint': -0.2})

    def take_picture(self, x, y, angle, i):

        self.omni_base.go_abs(x, y, angle, 1000)
        self.picture_pos()

        self.whole_body.move_to_joint_positions({'head_pan_joint': -math.pi/2 + math.pi/9})
        result = cv2.imwrite(f"kitchen_image_{self.formatted_datetime}_{i}_-20.png", self.cv2_img)
        if not result:
            rospy.logerr("Error while saving the image.")
            return "error"

        self.whole_body.move_to_joint_positions({'head_pan_joint': -math.pi/2})
        result = cv2.imwrite(f"kitchen_image_{self.formatted_datetime}_{i}_0.png", self.cv2_img)
        if not result:
            rospy.logerr("Error while saving the image.")
            return "error"

        self.whole_body.move_to_joint_positions({'head_pan_joint': -math.pi/2 - math.pi/9})
        result = cv2.imwrite(f"kitchen_image_{self.formatted_datetime}_{i}_20.png", self.cv2_img)
        if not result:
            rospy.logerr("Error while saving the image.")
            return "error"