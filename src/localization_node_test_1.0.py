#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved


# TITLE: TRIGGER FUNCTION TO START LOCALIZATION? PATH PLANNING AND PUBLISH WHEEL CMD

# DESCRIPTION:
# This script takes input from apriltags_postprocessing_node and checks if a localizable AT (that is included in our predefined map) is reached
# If so, it produces a usable input to the path_planning module for further processing and therefore acts as a switch, if not, AT_detection and lane_following continue
# The path_planning module returns the shortest path and executable wheel commands that are then published to unicorn_intersection_node

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, BoolStamped, AprilTagDetection, TurnIDandType

from path_planning_class import PathPlanner
#from visual_odometry import StateEstimation


# INITIATE DTROS CLASS (incl sub/pub)
class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LocalizationNode, self).__init__(node_name=localization_node)

        # Initialize variables
        self.node_name = rospy.get_name()
        self.veh_name = rospy.get_namespace().strip("/")
        self.AT = False

        # Initialize logging services
        self.log = rospy.loginfo() #correct?
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Get arrival point (from roslaunch/docker cmd setting the parameter 'goal_input' in terminal)
        self.input = rospy.get_param('~goal_input') # of type [id, distance before final AT]
        rospy.set_param('~goal_input', self.goal) # Write to parameter server for transparancy
        self.goal = self.input[0]
        self.goal_distance = self.input[1]

        # Import from external class PathPlanner as pp
        self.pp = PathPlanner()
        self.tags = self.pp.tags
        self.graph = self.pp.graph

        # List subscribers
        self.sub_AT_detection = rospy.Subscriber('~apriltags_out', AprilTagsWithInfos, self.callback) #from apriltags_postprocessing_node
        #self.sub_wheels_cmd = rospy.Subscriber("~/%s/camera_node/image/compressed" % veh_name, CompressedImage, self.odometer) #from camera_node

        # List publishers
        self.pub_direction_cmd = rospy.Publisher('~turn_id_and_type', TurnIDandType, queue_size = 1, latch = True) # to unicorn_intersection_node
        #self.pub_wheels_cmd = rospy.Publisher("~/%s/wheels_driver_node/wheels_cmd" % veh_name, WheelsCmdStamped, queue_size=1) # directly publish to wheels

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)


# CODE GOES HERE

    def callback(self, msg):
        #correct? Should not matter - ask to implement clean code ~with FSM
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":

            # Define path and cmd list from localized, current point in time
            trigger, starting_point = self.localization(msg)
            path, cmd = self.path_planning(trigger, starting_point)

            # If there is an actual path to be executed, run the following (path-actuation):
            if len(path) > 2:
                # from generated SP, generate sequence of wheel command of type TurnIDandType to publish
                wheelcommand = self.pathProcessor(path, cmd)
                self.pub_direction_cmd.publish(wheelcommand)

            # After the turn cmd of the before-last AT, start state-estimation (due to definition arrival point B):
            elif len(path) == 2:
                # Import from another class (since this might improve in future project - allow easy iteration)
                print("start state estimation")
                # After stopping, shutdown node

            # If the final AT is detected (could happen when arrival point B is too close to final AT), ignore input:
            else:
                pass #correct? is this necessary? Is there a state-switch? Can we not send any command at all to the publish-topic?


    def localization(self, msg):
        # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos

        for item in msg.detections:
            # WARNING: make sure only one id in Tags is detected, else vector size is > 1 --> account for in DT set up
            # If AT id is part of self.tags, continue with localization, else pass
            if item.id in self.tags:
                # Detect localizable AT
                self.AT = True
                self.log('AT successfully detected, localized duckiebot in map ... proceeding to path planning')

                # Return usable input for path_planning (graph is based on string variables)
                starting_point = str(item.id)
                return self.AT, starting_point

            # IGNORE: current map assumes only readible AT's - use as set-up for future project
            else:
                # WARNING: This needs to be updated if e.g. other apriltags are implemented in DT, and another command needs to be executed. Keep lane_following and AT_detection alive
                # Idea is to have another node subscribe to this topic, and return an action if AT is NOT part of self.tags, and don't do anything if otherwise
                self.AT = False
                starting_point = "No readible AT encountered"
                self.log(starting_point)
                return self.AT, starting_point #correct? or should this be pass/break?


    def path_planning(self, AT, starting_point):
        # Calculate shortest path from current AT in time

        if AT == False:
            print(starting_point)
            pass #correct? Keep lane_following and AT_detection alive

        else:
            self.log('Starting path planning')
            print("... starting path planning")
            # Define start and end point (dynamic)
            start = starting_point
            goal = self.goal

            # Run path planning from imported class (defined in initializer)
            path, cmd = self.pp.dijkstra(start, goal)
            rospy.loginfo('Confirmation: Path successfully generated !')
            return path, cmd


    def pathProcessor(self, path, cmd):
        # Process the incoming path and cmd lists and return image for publisher
        # WARNING: the path is updated every time an AT is detected - allows for dynamic changes in the map (when not static) or a new end goal, but adds to the computation time

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
            self.log("Turn left")
        elif cmd_comb[1] == 1:
            new_cmd.turn_type = 1
            self.log("Go straight")
        elif cmd_comb[1] == 2:
            new_cmd.turn_type = 2
            self.log("Turn right")

        return new_cmd

# SAFETY & EMERGENCY
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name)) #correct?


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    localization_node = LocalizationNode(node_name="localization_node")
    # Allow proper shutdown
    node = LocalizationNode()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()


# ----------------------------------------------------------------------------------------------------------------------------------------------------- #

# TO BE DONE:
# TODO: LOOK into node IND_NAV OF COORDINATION OR NAVIGATION FOR (DETECT RED LINE & AT DETECTION), THEN THERE IS A SWITCH TO TELL DB TO GO R, L OR STRAIGHT
# def cmdToWheel(self, AT_msg, wheelCmdList): publishes the relevant wheelcmd in runtime. Use pop out function to update the wheelcmd to be published # Stop localization node once 1st AT has come in?
# TODO: the actual execution should also take into account the traffic rules/lane directions - use another processing node to assign task priority
# TODO: # Look into coordinator node, unicorn intersection and random AT # rospy.loginfo, pop out function python

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
# Question_11: General functioning of else pass/break/continue statements. What is correct?
