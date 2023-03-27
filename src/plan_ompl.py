#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

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

# print(dir(ob.SpaceInformation))
# print(dir(og))
class Plan_OMPL:
    def __init__(self):

        self.StateValidityChecker = None
        # create an SE2 state space
        # self.space = ob.SE2StateSpace()

        vector_space = ob.RealVectorStateSpace(2)  # 2D vector space
        angle_space1 = ob.SO2StateSpace()  # 1D angle space 1
        angle_space2 = ob.SO2StateSpace()  # 1D angle space 2

        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-5)
        bounds.setHigh(5)
        vector_space.setBounds(bounds)

        self.space = ob.CompoundStateSpace()
        self.space.addSubspace(vector_space, 1.0)
        self.space.addSubspace(angle_space1, 0.5)
        self.space.addSubspace(angle_space2, 0.5)

        # set lower and upper bounds
        # bounds = ob.RealVectorBounds(2)
        # bounds.setLow(2.5)
        # bounds.setHigh(17.5)
        # self.space.setBounds(bounds)

        # create a simple setup object, note that si is needed when setting planner  
        self.si = ob.SpaceInformation(self.space)
        self.ss = og.SimpleSetup(self.si)

    def setStateValidityChecker(self, StateValidityChecker):
        self.StateValidityChecker = StateValidityChecker
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.StateValidityChecker))
    
    def set_planner(self):
        self.planner = og.RRTstar(self.si)
        self.ss.setPlanner(self.planner)

    def generate_valid_start_goal(self):
        start = ob.State(self.space)
        while True:
            start.random()
            if self.StateValidityChecker(start()):
                break
        goal = ob.State(self.space)
        while True:
            goal.random()
            if self.StateValidityChecker(goal()):
                break
        return start, goal

    def solve_planning_2D(self, start, goal, time_lim=10, simple=False):
        self.ss.setStartAndGoalStates(start, goal)
        path = []
        solved = self.ss.solve(time_lim)
        if solved:
            # try to shorten the path
            if simple:
                self.ss.simplifySolution()
            # print the simplified path
            # print(self.ss.getSolutionPath())

        #     states = self.ss.getSolutionPath().getStates()
        #     path_len = len(states)
        #     for i in range(path_len):
        #         X = states[i].getX()
        #         Y = states[i].getY()
        #         Yaw = states[i].getYaw()
        #         path.append([X, Y, Yaw])
        return solved, path

