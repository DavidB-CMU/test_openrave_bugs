#!/usr/bin/env python

#
# Test if cloning an environment causes a self-collision error
# for the fridge
#

import openravepy
import prpy
import prpy.planning # OMPLPlanner, HauserParabolicSmoother
import prpy.util # IsAtActiveDOFConfiguration(), IsAtTrajectoryEnd()
import herbpy

from tf import transformations # rotation_matrix(), concatenate_matrices()

import os
import sys
import numpy
import time

# Initialize OpenRAVE robot of type herbpy.herbrobot.HERBRobot:
#env, robot = herbpy.initialize(sim=True, attach_viewer=True)
#env, robot = herbpy.initialize(sim=True, attach_viewer='qtcoin')
env, robot = herbpy.initialize(sim=True, attach_viewer='rviz')
#env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')

# Set 7 DOF of the right arm active (default is full 26 DOF)
robot.SetActiveDOFs(robot.right_arm.GetArmIndices())
robot.SetActiveManipulator(robot.right_arm)

# Find the pr_ordata/kitchen directory:
from catkin.find_in_workspaces import find_in_workspaces
package_name = 'pr_ordata'
directory = 'data/kitchen'
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)
if len(objects_path) == 0:
    print('Can\'t find directory %s/%s' % (package_name, directory))
    sys.exit()
else:
    print objects_path # for me this is '/home/USERNAME/catkin_workspaces/herb_ws/src/pr-ordata/data/objects'
    objects_path = objects_path[0]

# Load the kitchen environment
kitchen_file = os.path.join(objects_path, 'pr_kitchen.env.xml')
env.Load(kitchen_file)

# Remove the wall kinbody
env.Remove(env.GetKinBody('walls'))

# Move the robot near the fridge
Rot_z = transformations.rotation_matrix(numpy.pi/4.0, (0,0,1))
trans = numpy.array([[1., 0., 0., 0.2],
                     [0., 1., 0., 0.5],
                     [0., 0., 1., 0.0],
                     [0., 0., 0., 1.]])
robot_pose = transformations.concatenate_matrices(trans, Rot_z)
robot.SetTransform(robot_pose)

# Get transform of the lower handle link,
# this is actually at the origin of the fridge
fridge = env.GetRobot('refrigerator')
fridge_handle = fridge.GetLink('lower_handle')
T_lower_handle = fridge_handle.GetTransform()

# Translate the lower handle axis to be at the actual handle
height = 0.925
forward = 0.427
sideways = -0.308
T_handle_offset = numpy.eye(4)
T_handle_offset[0:3,3] = numpy.array([forward, sideways, height])
T_handle = transformations.concatenate_matrices(T_lower_handle, T_handle_offset)

# Rotate the handle axis to suit the grasp approach direction,
# for HERB's gripper, z-axis is forward from palm
# and x-axis is towards two fingers
R_x = transformations.rotation_matrix(numpy.pi/2.0, (1,0,0))
T_handle_approach = transformations.concatenate_matrices(T_handle, R_x)
h2 = openravepy.misc.DrawAxes(env, T_handle_approach, dist=0.1, linewidth=0.5)

# Find a pre-grasp transform
T_pregrasp_offset = numpy.eye(4)
back_from_handle = -0.3
away_from_fridge = 0.18
T_pregrasp_offset[0:3,3] = numpy.array([away_from_fridge, 0.0, back_from_handle])
T_pregrasp = transformations.concatenate_matrices(T_handle_approach, T_pregrasp_offset)
h3 = openravepy.misc.DrawAxes(env, T_pregrasp, dist=0.2, linewidth=0.5)

# Close fingers 1/4 and spread fingers
a = numpy.pi/4.0
robot.right_arm.hand.MoveHand(f1=a, f2=a, f3=a, spread=numpy.pi)
time.sleep(0.1)

# Plan to pre-grasp pose
try:
    print "\nPlanToEndEffectorPose\n"
    path = robot.PlanToEndEffectorPose(T_pregrasp, timelimit=30.0)
except Exception as e:
    print("Failed to plan to pre-grasp pose")
    print e
    sys.exit()
# Smooth and time the path
smoother = prpy.planning.retimer.HauserParabolicSmoother(do_shortcut=True, do_blend=True)
traj = smoother.RetimeTrajectory(robot, path)
# Move the arm
print "\nExecuting trajectory... \n"
robot.ExecuteTrajectory(traj, defer=False, timeout=None) # False = wait until traj is executed


# Move gripper until it touches fridge door
try:
    print "\nMoveUntilTouch \n"
    robot.right_arm.MoveUntilTouch([1,0,0], 0.085, max_distance=None, max_force=5.0, max_torque=None, ignore_collisions=[fridge], velocity_limit_scale=0.25)
except Exception as e:
    print("Failed to MoveUntilTouch")
    print e
    sys.exit()


fridge = env.GetRobot('refrigerator')
print "\nBefore planning, fridge.CheckSelfCollision() = ", fridge.CheckSelfCollision(), "\n"


# Try to move gripper into handle
try:
    print "\nPlanToEndEffectorOffset (#2)\n"
    path = robot.PlanToEndEffectorOffset([0,1,0], 0.08, max_distance=None, timelimit=5.0) # THIS IS IN THE WORLD ORIGIN FRAME !!

    print "\nPlanning succeeded \n"
except Exception as e:
    print("Failed to plan to end effector offset (#2)")
    print e
    #sys.exit()


fridge = env.GetRobot('refrigerator')
print "\nAfter planning, fridge.CheckSelfCollision() = ", fridge.CheckSelfCollision(), "\n"


# we do this so the viewer doesn't close when the example is done
import IPython;
IPython.embed()
