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

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetection, TurnIDandType
from duckietown_msgs.msg import BoolStamped
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

        # List publishers
        self.pub_direction_cmd = rospy.Publisher('/%s/random_april_tag_turns_node/turn_id_and_type' %self.veh_name, TurnIDandType, queue_size = 1) # to unicorn_intersection_node
        self.pub_wheels_cmd = rospy.Publisher("/%s/wheels_driver_node/wheels_cmd" %self.veh_name, WheelsCmdStamped, queue_size = 1) # for emergency stop, else use onShutdown
        self.pub_override_joystick = rospy.Publisher('/%s/joy_mapper_node/joystick_override' %self.veh_name, BoolStamped, queue_size = 1)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)


# CODE GOES HERE

    def callback(self, msg):
        # Left out FSM statement
        # Insert joystick override command
        #override_msg = BoolStamped()
        #override_msg.data = False
        #self.pub_override_joystick.publish(override_msg)

        # Define path and cmd list from localized, current point in time
        trigger, starting_point = self.localization(msg)

        if trigger == True:
            path, cmd = self.path_planning(trigger, starting_point)

            # If there is an actual path to be executed, run the following (path-actuation):
            if len(path) > 1: #change to 2
                # from generated SP, generate sequence of wheel command of type TurnIDandType to publish
                wheelcommand = self.pathProcessor(path, cmd)
                self.pub_direction_cmd.publish(wheelcommand)
                # Concern: how to go back to ind_nav? This keeps running, so after cmd has been published and executed, it goes back to ind_nav

            # After the turn cmd of the before-last AT, start state-estimation (due to definition arrival point B):
            elif len(path) == 1:
                # Publish final turn command
                wheelcommand = self.pathProcessor(path, cmd)
                self.pub_direction_cmd.publish(wheelcommand)

                # TEST SPECIFIC
                time.sleep(5) # wait for 5s before shutting down
                self.pub_wheels_cmd.publish(self.stopCmd)

                # Publish trigger to wheel_odometry node (starts counting when first yellow striped is found)
                # WARNING: to activate a callback function on a topic inside another callback function on a different topic is not possible - use 2 scripts/nodes
                #se_cmd = DuckieSensor() #currently a random msg structure to visual_odometry_node, change TODO
                #se_cmd.value = self.goal_distance
                #se_cmd.is_analog = True
                #self.pub_state_estimation.publish(se_cmd)

                # Keep node alive
                #self.odometer(self.goal_distance, self.imageProcessor(self.img))
                #print("start state estimation")

                # After stopping, shutdown node
                #pass #correct? No cmd is sent to the unicorn_intersection_node anymore, only directly to the wheels

            # If the final AT is detected (could happen when arrival point B is too close to final AT), ignore input:
            else:
                # Stop immediately
                self.pub_wheels_cmd.publish(self.stopCmd)
        else:
            pass


    def localization(self, msg):
        # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos

        for item in msg.detections:
            # WARNING: make sure only one id in Tags is detected, else vector size is > 1 --> account for in DT set up
            # NOTE: this should be accounted for by the apriltags_postprocessing_node by taking the closest AT
            if item.id in self.tags:
                # Detect localizable AT
                self.AT = True

                # Return usable input for path_planning
                starting_point = item.id
                rospy.loginfo('Readible AT [%s] successfully detected, localized Duckiebot ... proceeding to path planning' %str(starting_point))
                return self.AT, starting_point

            # IGNORE:
            # NOTE: the current map assumes only readible AT's - so only use the following as set-up for future project
            else:
                # WARNING: This needs to be updated if e.g. other apriltags are implemented in DT, and another command needs to be executed. Keep lane_following and AT_detection alive
                # Idea is to have another node subscribe to this topic, and return an action if AT is NOT part of self.tags, and don't do anything if otherwise (previous case)
                self.AT = False

                outlying_point = item.id
                rospy.loginfo('Outlying AT [%s] detected, ignore message ... TODO: resolve issue by publishing to another node' %str(outlying_point))
                return self.AT, outlying_point #correct? or should this be pass/break?


    def path_planning(self, AT, starting_point):
        # Calculate shortest path from current AT in time

        if AT == False:
            rospy.loginfo('No usable AT detected, ignore message ... ')
            pass #correct? Keep lane_following and AT_detection alive

        else:
            rospy.loginfo('Starting path planning')

            # Define start and end point (dynamic)
            start = starting_point
            goal = self.goal

            # Run path planning from imported class (defined in initializer)
            path, cmd = self.pp.dijkstra(start, goal)
            rospy.loginfo('Confirmation: Path successfully generated ! With starting point the detected AT id %s' %str(start))
            return path, cmd


    def pathProcessor(self, path, cmd):
        # Process the incoming path and cmd lists and return image for publisher
        # WARNING: the path is updated every time an AT is detected
        # NOTE: this allows for dynamic changes in the map (when not static) or a new end goal, but adds to the computation time

        # Take current node and command
        cmd_comb = [path[0], cmd[0]]
        # Set up outgoing message type
        new_cmd = TurnIDandType()
        new_cmd.tag_id = cmd_comb[0]

        # Define the turn command
        # WARNING: unnecessary code, but allows more flexibility for future projects (different indices)
        # The index 0, 1 and 2 is used for turn_type msg type (see also random_april_tag_turns_node and open_loop_intersection_node)
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
