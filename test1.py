#!/usr/bin/env python

#
# Test if adjacent links are calculated correctly
#

import openravepy
import prpy
import herbpy

import os # path

def CheckAdjacentLinks(robot):
    """
    Check the link adjacency for an OpenRAVE robot.

    For a robot with N links, the number of adjacent link pairs + the
    number of non-adjacent pairs should equal N x (N-1)/2
    """
    fridge_links = robot.GetLinks()
    num_fridge_links = len(fridge_links)
    expected_num_link_pairs = num_fridge_links * (num_fridge_links - 1.0)/2.0

    adjacent_links = robot.GetAdjacentLinks()
    non_adjacent_links = robot.GetNonAdjacentLinks()
    num_adjacent_links = len(adjacent_links)
    num_non_adjacent_links = len(non_adjacent_links)
    actual_num_link_pairs = num_adjacent_links + num_non_adjacent_links

    print "robot.GetAdjacentLinks() = ", robot.GetAdjacentLinks()
    print "robot.GetNonAdjacentLinks() = ", robot.GetNonAdjacentLinks()

    if expected_num_link_pairs == actual_num_link_pairs:
        print("\nLink pairs are correct\n")
    else:
        print("\nERROR: some link pairs are missing\n")


# Initialize OpenRAVE robot of type herbpy.herbrobot.HERBRobot:
#env, robot = herbpy.initialize(sim=True, attach_viewer=True)
#env, robot = herbpy.initialize(sim=True, attach_viewer='qtcoin')
env, robot = herbpy.initialize(sim=True, attach_viewer='rviz')
#env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')


# Load the fridge (trimesh only)
current_dir = os.path.dirname(os.path.abspath(__file__))
#fridge_file = os.path.join(current_dir, 'prkitchen_refrigerator.robot.xml')
fridge_file = os.path.join(current_dir, 'fridge_trimesh.robot.xml')
env.Load(fridge_file)
fridge = env.GetKinBody('refrigerator')

CheckAdjacentLinks(fridge)

# Delete the fridge
env.Remove(env.GetKinBody('refrigerator'))


# Load the fridge (spheres only)
current_dir = os.path.dirname(os.path.abspath(__file__))
fridge_file = os.path.join(current_dir, 'fridge_spheres.robot.xml')
env.Load(fridge_file)
fridge = env.GetKinBody('refrigerator')

CheckAdjacentLinks(fridge)


import IPython;
IPython.embed()