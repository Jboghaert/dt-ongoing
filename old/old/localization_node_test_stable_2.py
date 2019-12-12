#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved


# TITLE: TRIGGER FUNCTION TO START LOCALIZATION? PATH PLANNING AND PUBLISH WHEEL CMD

# DESCRIPTION:
# This script takes input from apriltags_postprocessing_node and checks if a localizable AT (that is included in our predefined map) is reached
# If so, it produces a usable input to the path_planning module for further processing and therefore acts as a switch, if not, AT_detection and lane_following continue
# The path_planning module returns the shortest path and executable wheel commands that are then published to unicorn_intersection_node
# If the final AT is reached, a state estimation function (listening to the camera_node topic) is started (issue: actions wrt. both subscribers are coupled)

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import yaml
import time
import math

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetection, TurnIDandType
from duckietown_msgs.msg import BoolStamped, FSMState
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

from path_planning_class import PathPlanner


# INITIATE DTROS CLASS (incl sub/pub)
class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LocalizationNode, self).__init__(node_name=node_name)

        # Initialize variables
        self.node_name = "localization_node_test"
        self.veh_name = "maserati4pgts"
        self.AT = False
        self.turn_type = -1

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Get arrival point (from roslaunch/docker cmd setting the parameter 'goal_input' in terminal)
        self.goal = rospy.get_param('/%s/goal_input' %self.node_name) # of type final AT id [int32]
        self.goal_distance = rospy.get_param('/%s/goal_distance' %self.node_name) # of type [distance after 2nd to last AT (actually take stopline)]

        # Import from external class PathPlanner as pp, StateEstimator as se
        self.pp = PathPlanner()
        self.tags = self.pp.tags
        self.graph = self.pp.graph

        # List subscribers
        self.sub_AT_detection = rospy.Subscriber('/%s/apriltags_postprocessing_node/apriltags_out' %self.veh_name, AprilTagsWithInfos, self.callback) #from apriltags_postprocessing_node
        self.sub_mode = rospy.Subscriber('/%s/fsm_node/mode' %self.veh_name, FSMState, self.cbMode, queue_size=1)

        # List publishers
        self.pub_direction_cmd = rospy.Publisher('/%s/random_april_tag_turns_node/turn_id_and_type' %self.veh_name, TurnIDandType, queue_size = 1) # to unicorn_intersection_node
        self.pub_wheels_cmd = rospy.Publisher("/%s/wheels_driver_node/wheels_cmd" %self.veh_name, WheelsCmdStamped, queue_size = 1) # for emergency stop, else use onShutdown
        self.pub_override_joystick = rospy.Publisher('/%s/joy_mapper_node/joystick_override' %self.veh_name, BoolStamped, queue_size = 1)
        self.pub_turn_type = rospy.Publisher("/%s/turn_type" %self.veh_name, Int16, queue_size=1, latch=True)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(30)


# CODE GOES HERE
    def cbMode(self, mode_msg):
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)


    def callback(self, msg):
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
            # loop through list of april tags

            rospy.loginfo('Incoming AprilTagsWithInfos msg, starting callback ...')
            # filter out the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(msg.infos):
                if(taginfo.tag_type == taginfo.SIGN):
                    tag_det = (msg.detections)[idx]
                    pos = tag_det.pose.pose.position
                    distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                    if distance < dis_min:
                        dis_min = distance
                        idx_min = idx
                else:
                    continue

            # continue iff
            if idx_min != -1:
                taginfo = (msg.infos)[idx_min]

                rospy.loginfo('Starting localization module ...')
                # Define path and cmd list after localization of AT if AT went through the above filter first
                trigger, starting_point = self.localization(msg)

                if trigger == True:
                    path, cmd = self.path_planning(trigger, starting_point)

                    if len(path) > 1: #change to 2
                        # from generated SP, generate sequence of wheel command of type TurnIDandType to publish
                        turn_cmd = self.pathProcessor(path, cmd)
                        self.pub_direction_cmd.publish(turn_cmd)
                        # Publish turn_type
                        turn_cmd.turn_type = self.turn_type
                        self.pub_turn_type.publish(self.turn_type)
                        rospy.loginfo("Published turn_cmd and turn_type")

                    elif len(path) == 1:
                        # Publish final turn command
                        turn_cmd = self.pathProcessor(path, cmd)
                        self.pub_direction_cmd.publish(turn_cmd)
                        # Publish turn_type
                        turn_cmd.turn_type = self.turn_type
                        self.pub_turn_type.publish(self.turn_type)
                        rospy.loginfo("Published turn_cmd and turn_type")
                        # Wait for 5s before shutting down/state estimation
                        time.sleep(5)
                        self.pub_wheels_cmd.publish(self.stopCmd)
                        rospy.loginfo("Published stop_cmd")

                    else:
                        # Stop immediately
                        self.pub_wheels_cmd.publish(self.stopCmd)
                        rospy.loginfo("Published stop_cmd")


    def localization(self, msg): # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos
        for item in msg.detections:
            if item.id in self.tags: #check if closest AT is localizable
                self.AT = True
                # Return usable input for path_planning
                starting_point = item.id
                rospy.loginfo('Readible AT [%s] successfully detected, localized Duckiebot ... proceeding to path planning' %str(starting_point))
                return self.AT, starting_point
            else:
                self.AT = False
                outlier = item.id
                rospy.loginfo('No usable AT [%s] detected, passing FALSE' %str(item.id))
                return self.AT, outlier


    def path_planning(self, AT, starting_point): # Calculate shortest path from current AT in time
        if AT == True:
            rospy.loginfo('Starting path planning')
            # Define start and end point (dynamic)
            start = starting_point
            goal = self.goal
            # Run path planning from imported class (defined in initializer)
            path, cmd = self.pp.dijkstra(start, goal)
            rospy.loginfo('Confirmation: Path successfully generated ! With starting point the detected AT id %s' %str(start))
            return path, cmd
            

    def pathProcessor(self, path, cmd):
        # Take current node and command
        cmd_comb = [path[0], cmd[0]]
        # Set up outgoing message type
        new_cmd = TurnIDandType()
        new_cmd.tag_id = cmd_comb[0]

        if cmd_comb[1] == 0:
            new_cmd.turn_type = 0
            rospy.loginfo("Turn left")
        elif cmd_comb[1] == 1:
            new_cmd.turn_type = 1
            rospy.loginfo("Go straight")
        elif cmd_comb[1] == 2:
            new_cmd.turn_type = 2
            rospy.loginfo("Turn right")
        return new_cmd


# REACHING FINAL POINT
    def stopCmd(self):
        # Produce wheel stopping cmd vel(0,0)
        stop_cmd = WheelsCmdStamped()
        stop_cmd.vel_left = 0.0
        stop_cmd.vel_right = 0.0
        return stop_cmd


# SAFETY & EMERGENCY
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name)) #correct?
        self.pub_wheels_cmd.publish(self.stopCmd)


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = LocalizationNode(node_name="node_name")
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
