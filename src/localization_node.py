#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved


# TITLE
# TRIGGER FUNCTION FOR LOCALIZATION AFTER FIRST AT DETECTION
# TODO: LOOK into node IND_NAV OF COORDINATION OR NAVIGATION FOR (DETECT RED LINE & AT DETECTION), THEN THERE IS A SWITCH TO TELL DB TO GO R, L OR STRAIGHT

# TODO: if first apriltag is detected, go to path planning, if second AT is detected, go to other part of the code where wheel commands based on the generated path are published to wheel_cmd_node
# def pathProcessor(self, path): generates the wheelcommands from the path (wheelCmdList as output array)
# def cmdToWheel(self, AT_msg, wheelCmdList): publishes the relevant wheelcmd in runtime. Use pop out function to update the wheelcmd that needs to be published
# Stop localization node once 1st AT has come in

# WARNING: TODO the actual execution should also take into account the traffic rules/lane directions - use another processing node to assign task priority


# DESCRIPTION
# This script takes input from apriltags_postprocessing_node and checks if a localizable AT (that is included in our predefined map) is reached
# If so, it produces a usable input to the path_planning module for further processing and therefore acts as a switch, if not, AT_detection and lane_following continue


# OVERALL SOLUTION/PIPELINE
# First part of GOTO-1 solution:
# Activate indefinite_navigation (https://github.com/duckietown/dt-core/tree/daffy/packages/indefinite_navigation) OR lane_following () in parallel to AT_detection () and this node.
# When AT detected, localize in predefined map (according to DT protocol), and continue to path_planning


# QUESTIONS
# Question_1: where to implement? Implement in AT_detection_node and then send message to stop lane_following and run localization_node?
# Guess not, subscribe to output message (topic) of AT_detection_node, then run cmd 'if some_msg==True' and trigger the switch node to run localization_node.
# Question_2: how to run this node in parallel with AT_detection and lane_following (note: all incl in indefinite_navigation demo)?
# Guess building a new container, running it upon start (order of activating the containers matters?!)
# Question_3: Define a new topic to send to localization_node? Or could we use an existing one?
# Guess yes, since this is a new node/script, sending a message to another new script as trigger switch msg
# Reconsider, implement the whole localization node in the same script
# Question_4: when and how to abort/kill this node? Should only run once to generate path, then, other detected AT's should invoke a wheel_cmd only.
# Guess output cmd of this node can be published to other node, no need for storing these on ParameterServer.
# Question_5: what does DB do during computation of path_planning and localization? Does it proceed with ind_nav or lane_following?
# If yes, this is undesired as the executing wheel command could be activated if the DB has e.g. already passed the intersection..
# Question_6: more interesting to define 'map' as an array outside this script, and then calling it? More universal approach and scalable!
# Guess yes.

# Question_7: what does this node do when there is no AT detected? There is no message being published from apriltags_postprocessing_node?
# Guess nothing, since there is also no trigger for the callback function defined in this localization node: there is no action.
# Question_8: we need to override and pass an ignore command to the wheels/something when another AT is detected
# (and is understood by the node reading from the apriltags_postprocessing_node). Guess yes, so deactivate and replace that node in TOTAL.
# Question_9: how to make sure that the wheel_cmd is executed until the turn (L,R) or straight command is executed and a new road is entered
# ({road} = {drivable DT}\{intersections})? Guess, check existing code structures.

# Question_10: is the parameter self.goal updated everytime it is called for? s.t. during the deployment of the goto-1 solution,
# the end goal can be updated from the command line? Guess yes.

# ------------------------------------------------------------------------------------------------------------------------------------ #

# IMPORT
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, BoolStamped, AprilTagDetection, TurnIDandType

from path_planning_class import PathPlanner


# INITIATE DTROS CLASS (incl sub/pub)
class LocalizationNode(DTROS):

    def __init__(self, node_name):
        #Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LocalizationNode, self).__init__(node_name=localization_node)

        # Initialize variables
        self.node_name = rospy.get_name()
        self.log = rospy.loginfo() #correct?
        rospy.loginfo("[%s] Initializing." % (self.node_name))
        self.AT = False

        # Get arrival point (from roslaunch/docker cmd setting the parameter 'goal_input' in terminal)
        self.goal = rospy.get_param('~goal_input')
        rospy.set_param('~goal_input', self.goal) # Write to parameter server for transparancy

        # List all AT id's (int32[] type) that are considered for localization purposes in the predefined map
        # Import from external class PathPlanner as pp
        self.pp = PathPlanner()
        self.tags = self.pp.tags
        self.graph = self.pp.graph

        # List subscribers
        self.sub_AT_detection = rospy.Subscriber('~apriltags_out', AprilTagsWithInfos, self.callback) #from apriltags_postprocessing_node

        # List publishers
        #self.pub_override = rospy.Publisher("~/tag_detections", BoolStamped, queue_size = 1)
        #self.pub_pause = rospy.Publisher("~send_msg_to_lane_following_node", BoolStamped, queue_size = 1)
        self.pub_direction_cmd = rospy.Publisher('~turn_id_and_type', TurnIDandType, queue_size = 1, latch = True) # to unicorn_intersection_node

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)


# CODE GOES HERE
# Look into coordinator node, unicorn intersection and random AT
# rospy.loginfo, pop out function python

    def callback(self, msg):
        #correct? Should not matter - ask to implement clean code ~with FSM
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
            # Toplevel function that is invoked by subscriber and then publishes commands to relevant topic
            # Define path from localized starting point/or current point in time
            path = self.path_planning(self.localization(msg))

            # If there is an actual path to be executed, run the following (path-actuation):
            if len(path) > 1:
                # from generated SP, generate sequence of wheel command of type TurnIDandType to publish
                wheelcommand = self.pathProcessor(path)

                # publish relevant wheel command of type TurnIDandType
                self.pub_direction_cmd.publish(wheelcommand)

            # If there is no path, and the last AT is encountered, run the following (state-estimation):
            else:
                "Do something"

            #IGNORE: find when a single command should be published (and for how long)
            #IGNORE: this is not sufficient to allow enough time for wheelcmd execution (rospy.sleep(5.0))


    def localization(self, msg):
        # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos
        for item in msg.detections:
            # WARNING: make sure only one id in Tags is detected, else vector size is > 1 --> account for in DT set up
            # If AT id is part of self.tags, continue with localization, else pass
            if item.id in self.tags:
                # Detect localizable AT
                self.AT = True
                self.log('AT successfully detected, localized duckiebot in map')
                print("AT successfully detected, localized duckiebot in map ... now proceeding to path planning")

                # Return usable input for path_planning (graph is based on string variables)
                starting_point = str(item.id)
                return self.AT, starting_point

            else:
                # This needs to be updated if e.g. other apriltags are implemented in DT, and another command needs to be executed.
                # Idea is to have another node subscribe to this topic, and return an action if AT is NOT part of self.tags, and don't do anything if otherwise
                # Keep lane_following and AT_detection alive
                self.AT
                starting_point = "No readible AT encountered"
                self.log('No readible AT encountered')
                return self.AT, starting_point #correct? or should this be pass/break?


    def path_planning(self, AT, starting_point):
        # Run localization
        # AT, starting_point = self.localization(msg)
        # Only proceed if localized
        if AT == False:
            # Keep lane_following and AT_detection alive
            print(starting_point)
            pass #correct?

        else:
            self.log('Starting path planning')
            print("... starting path planning")
            # Copy starting point found in localization
            start = starting_point
            # Define arrival point from manual input in docker run cmd/terminal
            goal = self.goal
            # Define map (in terms of weights)
            graph = self.graph

            # Run path planning from imported class (defined in initializer)
            path = self.pp.dijkstra(start, goal)
            rospy.loginfo('Path successfully generated')
            print("Confirmation: path successfully generated !")
            return path


    def pathProcessor(self, path):
        # WARNING: the path is updated every time an AT is detected - allows for dynamic changes in the map (once this is not made static) or a new end goal, but adds to the computation time
        # Take second entry (cmd) from the first list from the newly iterated path
        cmd = path[0]
        # Set up outgoing message type
        new_cmd = TurnIDandType()
        new_cmd.tag_id = cmd[0]
        # Define the turn command
        # WARNING: could be hardcoded in the map as well, but current set up allows more flexibility
        if cmd[1] == 0:
            # The index 0, 1 and 2 is used for turn_type msg type (see also random_april_tag_turns_node and open_loop_intersection_node)
            new_cmd.turn_type = 0
            self.log("Turn left")
        elif cmd[1] == 1:
            new_cmd.turn_type = 1
            self.log("Go straight")
        elif cmd[1] == 2:
            new_cmd.turn_type = 2
            self.log("Turn right")

        return new_cmd

# SAFETY & EMERGENCY
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    localization_node = LocalizationNode(node_name="localization_node")
    # Allow proper shutdown
    node = LocalizationNode()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

