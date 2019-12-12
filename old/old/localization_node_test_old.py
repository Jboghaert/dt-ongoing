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
from duckietown_msgs.msg import BoolStamped, DuckieSensor
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

from path_planning_class import PathPlanner
from visual_odometry import StateEstimator


# INITIATE DTROS CLASS (incl sub/pub)
class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LocalizationNode, self).__init__(node_name=localization_node)

        # Initialize variables
        self.node_name = rospy.get_name()
        self.veh_name = rospy.get_namespace().strip("/")
        self.AT = False
        self.camera = False
        #self.image = None

        # Initialize logging services
        #self.log = rospy.loginfo() #correct?
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Get arrival point (from roslaunch/docker cmd setting the parameter 'goal_input' in terminal)
        self.input = rospy.get_param('~goal_input') # of type [final AT id, distance after 2nd to last AT (actually take stopline)]
        rospy.set_param('~goal_input', self.goal) # Write to parameter server for transparancy
        self.goal = self.input[0]
        self.goal_distance = self.input[1]

        # Import from external class PathPlanner as pp, StateEstimator as se
        self.pp = PathPlanner()
        self.tags = self.pp.tags
        self.graph = self.pp.graph
        #self.se = StateEstimator(stripe_length=2.5) #set length of midlane stripe in cm

        # List subscribers
        self.sub_AT_detection = rospy.Subscriber('~apriltags_out', AprilTagsWithInfos, self.callback) #from apriltags_postprocessing_node

        # List publishers
        self.pub_direction_cmd = rospy.Publisher('~turn_id_and_type', TurnIDandType, queue_size = 1, latch = True) # to unicorn_intersection_node
        #self.pub_state_estimation = rospy.Publisher('~state_estimator', BoolStamped, queue_size = 1, latch = True) # create new topic to visual_odometry
        self.pub_wheels_cmd = rospy.Publisher("~/%s/wheels_driver_node/wheels_cmd" % veh_name, WheelsCmdStamped, queue_size=1) # for emergency stop, else use onShutdown

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)


# CODE GOES HERE

    def callback(self, msg):
        # Left out FSM statement

        # Define path and cmd list from localized, current point in time
        trigger, starting_point = self.localization(msg)
        path, cmd = self.path_planning(trigger, starting_point)

        # If there is an actual path to be executed, run the following (path-actuation):
        if len(path) > 2:
            # from generated SP, generate sequence of wheel command of type TurnIDandType to publish
            wheelcommand = self.pathProcessor(path, cmd)
            self.pub_direction_cmd.publish(wheelcommand)
            # Concern: how to go back to ind_nav? This keeps running, so after cmd has been published and executed, it goes back to ind_nav

        # After the turn cmd of the before-last AT, start state-estimation (due to definition arrival point B):
        elif len(path) == 2:
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


    def localization(self, msg):
        # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos

        for item in msg.detections:
            # WARNING: make sure only one id in Tags is detected, else vector size is > 1 --> account for in DT set up
            # NOTE: this should be accounted for by the apriltags_postprocessing_node by taking the closest AT
            if item.id in self.tags:
                # Detect localizable AT
                self.AT = True
                rospy.loginfo('AT successfully detected, localized duckiebot in map ... proceeding to path planning')

                # Return usable input for path_planning
                starting_point = str(item.id) #incorrect, should be int32!
                return self.AT, starting_point

            # IGNORE:
            # NOTE: the current map assumes only readible AT's - so only use the following as set-up for future project
            else:
                # WARNING: This needs to be updated if e.g. other apriltags are implemented in DT, and another command needs to be executed. Keep lane_following and AT_detection alive
                # Idea is to have another node subscribe to this topic, and return an action if AT is NOT part of self.tags, and don't do anything if otherwise (previous case)
                self.AT = False
                starting_point = "No readible AT encountered"
                rospy.loginfo('Outlying AT detected, ignore message ... TODO: resolve issue by publishing to another node')
                return self.AT, starting_point #correct? or should this be pass/break?


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
            rospy.loginfo('Confirmation: Path successfully generated !')
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

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


"""
    def cbCamera(self, img):
        self.image = img


    def odometer(self, dist, img):
        # Only trigger when necessary
        if self.camera == False:
            pass #correct?

        else:
            # Define number of midlane stripes to cover until final point is reached from last AT
            # Used distance is actual distance, not input distance (which would be the distance BEFORE the last AT is reached)
            n_stripes = dist / self.stripe_length

            # Count number of stripes incoming
            n_stripes_actual = self.stripeCounter(img)

            # Trigger function
            if n_stripes_actual < n_stripes:
                rospy.loginfo("Still going ... whoop whoop")

            elif n_stripes_actual >= n_stripes:
                # Continue driving (tune by testing) as final destination is in front of DB (image ≠ actual position)
                rospy.loginfo("Reaching final destination point ... preparing x seconds delayed stop")
                seconds = (n_stripes_actual - n_stripes) + 2
                time.sleep(seconds)
                # Publish command
                self.pub_wheels_cmd.publish(self.stopCmd)
                rospy.loginfo("Reached final destination point ... sending stop_cmd")

            else:
                # Stop immediately, do not delay stopping procedure
                rospy.loginfo("Final destination point already reached ... preparing quick stop")
                # Publish command
                self.pub_wheels_cmd.publish(self.stopCmd)
                rospy.loginfo("Reached final destination point ... sending stop_cmd")
                rospy.loginfo("")
"""