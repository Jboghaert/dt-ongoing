#!/usr/bin/env python

# TITLE
# SHORTEST PATH PLANNER GIVEN INPUT AT, PREDEFINED MAP AND COST FUNCTION (distance, #turns)

# DESCRIPTION
# This script is called by and takes input from localization_node and calculates the shortest path (in our predefined map) based on Dijkstra's Algorithm
# It incorporates basic traffic rules, s.a. lane direction, no U-turns (see user scenario description of demo_map)
# It also takes into account the distances AND the number of turns between nodes (AT's) as weights/scaled costs for the SSP problem.s
# Output is a string of nodes (AT's) that the DB has to follow in order to get to the desired end point via the shortest path.

# TODO:
# Convert this to DTROS class based script, in order to use ros logging services

# IMPORT
import numpy as np
import os
import rospy


# INITIATE DTROS CLASS (incl sub/pub)
class PathPlanner:

    def __init__(self):
        # Use generated start and goal defined in localization_node
        #self.start = start
        #self.goal = goal

        # List all AT id's (int32[] type) that are considered for localization purposes in the predefined map
        self.tags = [id_1, id_2, id_3, id_4, id_5, id_6, id_7, id_8, id_9, id_10, id_11, id_12]
        self.number_of_tags = len(self.tags)

        # Make string from id codes/AT nodes (array starts from index 0)
        self.set_iD = []
        for i in range(0, self.number_of_tags - 1):
            self.set_iD[i] = str(self.tags[i])

        iD = self.set_iD

        """
        Use this version for the demo version (with AT implemented)
        self.graph = {iD[1]: {iD[2]: 10, iD[3]: 4},
                    iD[2]: {iD[4]: 8, iD[4]: 2},
                    iD[3]: {iD[2]: 6, iD[4]: 2, iD[5]: 2},
                    iD[4]: {iD[5]: 7},
                    iD[5]: {iD[4]: 9}}
        """

        # Define weights/costs between AT's to base SSP on - this is the predefined DT map (manual input)
        # Each intersection has as many different AT's as it has roads (e.g. a T-intersection has 3 roads, and for each road another AT id)
        # TODO: import as .yaml file from external source (have a map for each DT configuration, or configure from watchtowers)
        self.graph = {
                    'a1': {'b1': 4.7, 'c1': 2.7}, 
                    'a2': {'c1': 1.9, 'c1': 6.9}, 
                    'a3': {'b1': 5.4, 'c1': 7.6},

                    'b1': {'c2': 1.9, 'f1': 2}, 
                    'b2': {'a2': 3.9, 'c2': 1.9}, 
                    'b3': {'a2': 3.8, 'f1': 2.7},

                    'c1': {'b3': 1.9,  'd1': 2.7,  'g1':2}, 
                    'c2': {'a3': 2.7,  'd1': 2,    'g1':1.9},
                    'c3': {'a3': 2,    'b3': 2.7,  'd1':1.9},
                    'c4': {'a3': 1.9,  'b3': 2,    'd1':2.7}, 
​
                    'd1': {'e1': 3, 'h1': 1.9}, 
                    'd2': {'c4': 2.7, 'e1': 2.9},
                    'd3': {'c4': 2, 'h1': 2.7},

                    'e1': {'a1': 9.4, 'j3': 6.8}, 
                    'e2': {'d3': 2.9, 'j3': 6.9},
                    'e3': {'a1': 7.7, 'd3': 3.7},
​
                    'f1': {'g2': 2.7, 'i2': 4.7}, 
                    'f2': {'b2': 2,   'g2': 1.9},
                    'f3': {'b2': 1.9, 'i2': 5.4},
​
                    'g1': {'f3': 1.9,  'h2': 2.7,  'i1':2},
                    'g2': {'c3': 2.7,  'h2': 2,    'i1':1.9},
                    'g3': {'c3': 2,    'f3': 2.7,  'h2':1.9},
                    'g4': {'c3': 1.9,  'f3': 2,    'i1':2.7},
​
                    'h1': {'g4': 1.9, 'j1': 2},
                    'h2': {'d2': 2.7, 'j1': 1.9},
                    'h3': {'g4': 2.7, 'd2': 2},
​
                    'i1': {'f2': 3.8, 'j2': 2.7},
                    'i2': {'g3': 2.7, 'j2': 2},
                    'i3': {'f2': 3.9, 'g3': 1.9},
​
                    'j1': {'e3': 8.4, 'i3': 1.9},
                    'j2': {'e3': 7.7, 'h3': 2.7},
                    'j3': {'h3': 1.9, 'i3': 2}}

        self.graph_direction = {
                    'a1': {'b1': 1, 'c1': 0},
                    'a2': {'c1': 2, 'e2': 1},
                    'a3': {'b1': 0, 'e2': 2},

                    'b1': {'c2': 0, 'f1': 1},
                    'b2': {'a2': 1, 'c2': 2},
                    'b3': {'a2': 2, 'f1': 0},

                    'c1': {'b3': 2,  'd1': 0,  'g1':1},
                    'c2': {'a3': 0,  'd1': 1,  'g1':2},
                    'c3': {'a3': 1,  'b3': 0,  'd1':2},
                    'c4': {'a3': 2,  'b3': 1,  'g1':0},

                    'd1': {'e1': 1, 'h1': 2},
                    'd2': {'c4': 0, 'e1': 2},
                    'd3': {'c4': 1, 'h1': 0},

                    'e1': {'a1': 0, 'j3': 2},
                    'e2': {'d3': 2, 'j3': 1},
                    'e3': {'a1': 1, 'd3': 0},

                    'f1': {'g2': 0, 'i2': 1},
                    'f2': {'b2': 1, 'g2': 2},
                    'f3': {'b2': 2, 'i2': 0},

                    'g1': {'f3': 2,  'h2': 0,  'i1':1},
                    'g2': {'c3': 0,  'h2': 1,  'i1':2},
                    'g3': {'c3': 1,  'f3': 0,  'h2':2},
                    'g4': {'c3': 2,  'f3': 1,  'i1':0},

                    'h1': {'g4': 2, 'j1': 1},
                    'h2': {'d2': 0, 'j1': 2},
                    'h3': {'g4': 0, 'd2': 1},

                    'i1': {'f2': 2, 'j2': 0},
                    'i2': {'g3': 0, 'j2': 1},
                    'i3': {'f2': 1, 'g3': 2},

                    'j1': {'e3': 0, 'i3': 2},
                    'j2': {'e3': 1, 'h3': 0},
                    'j3': {'h3': 2, 'i3': 1}}


    def dijkstra(self, start, goal):
        # Set up
        shortest_distance = {}
        predecessor = {}
        infinity = 9999999
        path = []
        cmd = []

        for node in self.graph:
            shortest_distance[node] = infinity
        shortest_distance[start] = 0

        while graph:
            minNode = None
            for node in graph:
                if minNode is None:
                    minNode = node
                elif shortest_distance[node] < shortest_distance[minNode]:
                    minNode = node

            for childNode, weight in graph[minNode].items():
                if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                    shortest_distance[childNode] = weight + shortest_distance[minNode]
                    predecessor[childNode] = minNode
            self.graph.pop(minNode)

        currentNode = goal
        while currentNode != start:
            try:
                path.insert(0, currentNode)
                cmd.insert(0, self.graph_direction[predecessor[currentNode]][currentNode])
                currentNode = predecessor[currentNode]
            except KeyError:
                print('Path not reachable')
                break

        path.insert(0, start)
        if shortest_distance[goal] != infinity:
            print('Shortest distance is ' + str(shortest_distance[goal]))
            print('And the path is ' + str(path))
            print('And the sequence of wheel cmds is' + str(cmd))
            print('... waiting for confirmation from path_planning module ...')

        return path, cmd
        # WARNING: len(path) = (len(cmd) + 1) !!
        # returns sequence of nodes in string format, this string will be the str-version of the AT id in int32 format
        # this should allow easy calculation/transformation back and forth