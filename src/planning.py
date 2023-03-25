#!/usr/bin/env python3
# -*- encoding:utf-8 -*-


######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(
        0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

from env_robot import Env_Robot
from visualization import vis_for_2D_planning

er = Env_Robot()


def isStateValid(state):
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    global er
    flag = er.is_state_valid_2D(state)
    print(flag)
    return False


def planWithSimpleSetup():
    import time
    time1 = time.time()
    global er
    print(66666)
    path = []
    # create an SE2 state space
    space = ob.SE2StateSpace()

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(20)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(er.is_state_valid_2D))

    start = ob.State(space)
    # we can pick a random start state...
    while True:
        start.random()
        if er.is_state_valid_2D(start()):
            break
    # start().setX(14)
    # start().setY(14)
    # start().setYaw(0)
    # ... or set specific values
    # start().setX(.5)

    goal = ob.State(space)
    # we can pick a random goal state...
    while True:
        goal.random()
        if er.is_state_valid_2D(goal()):
            break
    # goal().setX(5)
    # goal().setY(5)
    # ... or set specific values
    # goal().setX(-.5)
    print("111")
    ss.setStartAndGoalStates(start, goal)

    # this will automatically choose a default planner with
    # default parameters
    print("start solving")
    print("time1", time.time()-time1)
    solved = ss.solve(20)
    print("time2", time.time()-time1)
    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print(ss.getSolutionPath())
        print("finish")
        states = ss.getSolutionPath().getStates()
        path_len = len(states)
        for i in range(path_len):
            X = states[i].getX()
            Y = states[i].getY()
            Yaw = states[i].getYaw()
            path.append([X, Y, Yaw])

    # for vis
    path_wit_robot = er.get_config_path_with_robot_info_2D(path)
    fig_file = "test"
    vis_for_2D_planning(rec_env=er.obstacles_vis, start=path_wit_robot[0], goal=path_wit_robot[-1], path=path_wit_robot,
                        size=20, pixel_per_meter=100, save_fig_dir=fig_file)
    print("time3", time.time()-time1)


def planTheHardWay():
    # create an SE2 state space
    space = ob.SE2StateSpace()
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    # set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    # create a random start state
    start = ob.State(space)
    start.random()
    # create a random goal state
    goal = ob.State(space)
    goal.random()
    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    # create a planner for the defined space
    planner = og.RRTConnect(si)
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)
    # perform setup steps for the planner
    planner.setup()
    # print the settings for this space
    print(si.settings())
    # print the problem settings
    print(pdef)
    # attempt to solve the problem within one second of planning time
    solved = planner.solve(1.0)

    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        print("Found solution:\n%s" % path)
    else:
        print("No solution found")

# def get_quaternion_from_euler(roll, pitch, yaw):
#     """
#     Convert an Euler angle to a quaternion.

#     Input
#       :param roll: The roll (rotation around x-axis) angle in radians.
#       :param pitch: The pitch (rotation around y-axis) angle in radians.
#       :param yaw: The yaw (rotation around z-axis) angle in radians.

#     Output
#       :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
#     """
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
#         np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
#         np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
#         np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
#         np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

#     return [qx, qy, qz, qw]

if __name__ == "__main__":
    planWithSimpleSetup()
    # print("")
    # planTheHardWay()
    # import fcl
    # import numpy as np
    # g1 = fcl.Box(0, 1, 0)
    # l1 = np.array([4, 0, 0])
    # q = get_quaternion_from_euler(0,0,0)
    # t1 = fcl.Transform(l1)
    # o1 = fcl.CollisionObject(g1, t1)
    # o1.se

    # g2 = fcl.Box(2, 2, 0)
    # t2 = fcl.Transform()
    # o2 = fcl.CollisionObject(g2, t2)

    # request = fcl.CollisionRequest()
    # result = fcl.CollisionResult()

    # ret = fcl.collide(o1, o2, request, result)
    # print(ret)

    # request = fcl.DistanceRequest()
    # result = fcl.DistanceResult()
    # dis = fcl.distance(o1, o2, request, result)
    # print(dis)
