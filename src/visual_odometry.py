#!/usr/bin/env python

# TITLE
# SHORTEST PATH PLANNER GIVEN INPUT AT, PREDEFINED MAP AND COST FUNCTION (distance, #turns)

# DESCRIPTION
# This script is called by and takes input from localization_node once the final AT before the final point B is reached
# and checks whether the Duckiebot should be stopped based on the distance that has been covered from this last AT. It
# therefore only outputs a stop command to the wheels and shuts down the DB with the message, 'destination reached'.
# Input is the final destination point of type [final AT id, distance before final AT]

# IMPORT
import numpy as np
import os
import rospy
import cv2
import time

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, BoolStamped, AprilTagDetection, TurnIDandType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge


# INITIATE DTROS CLASS
class StateEstimator(DTROS):

    def __init__(self, stripe_length):
        # Initialize DTROS class
        super(StateEstimator, self).__init__(node_name=visual_odometry) #correct?

        # Initialize variables
        self.node_name = rospy.get_name()
        self.veh_name = rospy.get_namespace().strip("/")
        veh_name = self.veh_name

        self.stripe_length = stripe_length #length of 1 midlane stripe in cm

        # Initialize logging services
        self.log = rospy.loginfo() #correct?
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # IGNORE: already included in main script
        self.input = rospy.get_param('~goal_input') # of type [id, distance before final AT]
        rospy.set_param('~goal_input', self.goal) # Write to parameter server for transparancy
        self.goal = self.input[0]
        self.goal_distance = self.input[1]

        # Allow for faster computation
        rospy.set_param('/%s/camera_node/res_w' % veh_name, 160)
        rospy.set_param('/%s/camera_node/res_h' % veh_name, 120)

        # List subscribers
        self.sub_wheels_cmd = rospy.Subscriber("~/%s/camera_node/image/compressed" % veh_name, CompressedImage, self.stripeCounter) #from camera_node

        # Conclude
        self.bridge = CvBridge()
        self.log("Initialized")


    def imageProcessor(self, image):
        # Convert image into readible (non-compressed) HSV image
        image = CvBridge().compressed_imgmsg_to_cv2(image)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask on yellow
        upper_yellow = np.array([112,80,80], np.uint8)
        lower_yellow = np.array([124,255,255], np.uint8)
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Split mask (exclude disturbances of e.g. duckiebots on right side)
        # TODO

        return mask


    def stripeCounter(self, image):
        image = self.imageProcessor(image)

        # count number of stripes seen
        n_stripes_actual = 'some code'
        # for static image: https://stackoverflow.com/questions/38619382/how-to-count-objects-in-image-using-python
        # for video: https://www.pyimagesearch.com/2018/08/13/opencv-people-counter/

        return n_stripes_actual


if __name__ == "__main__":
    # Initialize the node with rospy
    visual_odometry = StateEstimator(node_name="visual_odometry")
    # Keep it spinning to keep the node alive
    rospy.spin()


###-------------------------------------------------------------------------------------------------------------------
"""
        # Take incoming message and command from camera_node/image/compressed topic
        # Convert to usable image
        img = self.imageProcessor(img)
        h, s, v = img[:, :, 0], img[:, :, 1], img[:, :, 2]
        bright_left = np.sum(v[:, 0:160])
        bright_right = np.sum(v[:, 161:320])

        # Apply a mask, filter out yellow colour
        if bright_left > bright_right:
            print('Left is brighter --> turn right')
            u_l_limited, u_r_limited = self.speedToCmd(0.1, 1.5)
        else:
            print('Right is brighter --> turn left')
            u_l_limited, u_r_limited = self.speedToCmd(1.5, 0.1)


        return stop_cmd
        # WARNING: len(path) = (len(cmd) + 1) !!
        # returns sequence of nodes in string format, this string will be the str-version of the AT id in int32 format
        # this should allow easy calculation/transformation back and forth
"""
###--------------------------------------------------------------------------------------------------------------------
