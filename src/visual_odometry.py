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

    def __init__(self):
        # Initialize DTROS class
        super(StateEstimator, self).__init__(node_name=visual_odometry) #correct?

        # Initialize variables
        self.node_name = rospy.get_name()
        self.veh_name = rospy.get_namespace().strip("/")
        veh_name = self.veh_name

        self.stripe_length = 2.5 #length of 1 midlane stripe in cm
        self.from_localization_node = False

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
        self.sub_wheels_cmd = rospy.Subscriber("~/%s/camera_node/image/compressed" % veh_name, CompressedImage, self.odometer) #from camera_node
        self.sub_localization_node = rospy.Subscriber('~state_estimator', BoolStamped, self.callback)

        # List publishers
        self.pub_wheels_cmd = rospy.Publisher("~/%s/wheels_driver_node/wheels_cmd" % veh_name, WheelsCmdStamped, queue_size=1) # directly publish to wheels

        # Conclude
        self.bridge = CvBridge()
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)


    def callback(self, info):
        # Only invoke
        # TODO look into cbSwitch
        if info.is_analog == True:
            self.from_localization_node = True
            # self.goal_distance = info.value
            # Either define here over a topic, or directly via rospy.get_param
        else:
            pass #correct?


    def odometer(self, img):
        # Only trigger when necessary and last AT is reached
        while self.from_localization_node == True:
            # Define number of midlane stripes to cover until final point is reached from last AT
            # Used distance is actual distance, not input distance (which would be the distance BEFORE the last AT is reached)
            n_stripes = self.goal_distance // int(self.stripe_length)

            # Count number of stripes incoming (continously, so use 'while' loop)
            n_stripes_actual = self.stripeCounter(img)

            # Trigger function
            if n_stripes_actual < n_stripes:
                self.log("Still going ... whoop whoop")

            elif n_stripes_actual >= n_stripes:
                # Continue driving (tune by testing) as final destination is in front of DB (image ≠ actual position)
                self.log("Reaching final destination point ... preparing x seconds delayed stop")
                seconds = (n_stripes_actual - n_stripes) + 2
                time.sleep(seconds)
                # Publish command
                self.pub_wheels_cmd.publish(self.stopCmd)
                self.log("Reached final destination point ... sending stop_cmd")
                self.from_localization_node = False

            else:
                # Stop immediately, do not delay stopping procedure
                self.log("Final destination point already reached ... preparing quick stop")
                # Publish command
                self.pub_wheels_cmd.publish(self.stopCmd)
                self.log("Reached final destination point ... sending stop_cmd")
                self.from_localization_node = False

        else:
            pass #correct?


    def stripeCounter(self, image):
        # Convert image to readible input
        image = self.imageProcessor(image)

        # Now count number of stripes from continuous image stream
        n_stripes_actual = 2 #some int alue
        # for static image: https://stackoverflow.com/questions/38619382/how-to-count-objects-in-image-using-python
        # for video: https://www.pyimagesearch.com/2018/08/13/opencv-people-counter/

        return n_stripes_actual


    def imageProcessor(self, image):
        # Convert image into readible (non-compressed) HSV image
        image = CvBridge().compressed_imgmsg_to_cv2(image)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask on yellow
        upper_yellow = np.array([112,80,80], np.uint8)
        lower_yellow = np.array([124,255,255], np.uint8)
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        # Converge with image, make rest black for contrast

        # Split mask (exclude disturbances of e.g. duckiebots on right side)
        # TODO

        return mask


    def stopCmd(self):
        # Produce wheel stopping cmd vel(0,0)
        stop_cmd = WheelsCmdStamped()
        stop_cmd.vel_left = 0.0
        stop_cmd.vel_right = 0.0
        return stop_cmd


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

