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
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link

er = Env_Robot()


# print(dir(ob.SpaceInformation))
# print(dir(og))
class Plan_OMPL:
    def __init__(self, configure_type="Rigidbody_2D"):

        self.planner = None
        self.StateValidityChecker = None
        self.configure_type = configure_type

        if configure_type == "Rigidbody_2D":
            # create an SE2 state space
            self.space = ob.SE2StateSpace()
            # set bounds
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-15)
            bounds.setHigh(15)
            self.space.setBounds(bounds)
            # # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

        if configure_type == "Two_Link_2D":
            # state space components
            vector_space = ob.RealVectorStateSpace(2)  # 2D vector space
            angle_space1 = ob.SO2StateSpace()  # 1D angle space 1
            angle_space2 = ob.SO2StateSpace()  # 1D angle space 2
            bounds = ob.RealVectorBounds(2)  # set bounds on vector
            bounds.setLow(-15)
            bounds.setHigh(15)
            vector_space.setBounds(bounds)
            # create space
            self.space = ob.CompoundStateSpace()
            # weight 1.0 as default, set for computing distance
            self.space.addSubspace(vector_space, 1.0)
            self.space.addSubspace(angle_space1, 0.5)
            self.space.addSubspace(angle_space2, 0.5)

            # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

    # This function aims to solve problems when multiple plan with python using C++ ompl lib
    def reboot(self):
        if self.configure_type == "Rigidbody_2D":
            # create an SE2 state space
            self.space = ob.SE2StateSpace()
            # set bounds
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-15)
            bounds.setHigh(15)
            self.space.setBounds(bounds)
            # # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

        if self.configure_type == "Two_Link_2D":
            # state space components
            vector_space = ob.RealVectorStateSpace(2)  # 2D vector space
            angle_space1 = ob.SO2StateSpace()  # 1D angle space 1
            angle_space2 = ob.SO2StateSpace()  # 1D angle space 2
            bounds = ob.RealVectorBounds(2)  # set bounds on vector
            bounds.setLow(-15)
            bounds.setHigh(15)
            vector_space.setBounds(bounds)
            # create space
            self.space = ob.CompoundStateSpace()
            # weight 1.0 as default, set for computing distance
            self.space.addSubspace(vector_space, 1.0)
            self.space.addSubspace(angle_space1, 0.5)
            self.space.addSubspace(angle_space2, 0.5)

            # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

    def setStateValidityChecker(self, StateValidityChecker):
        self.StateValidityChecker = StateValidityChecker
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.StateValidityChecker))

    def set_planner(self):
        self.planner = og.RRTstar(self.si)
        # self.planner = og.MPN(self.si)
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

    def convert_ompl_config_to_list_config(self, state_ompl):
        if self.configure_type == "Rigidbody_2D":
            X = state_ompl().getX()
            Y = state_ompl().getY()
            Yaw = state_ompl().getYaw()
            return [X, Y, Yaw]

        if self.configure_type == "Two_Link_2D":
            Vec_X = state_ompl[0]
            Vec_Y = state_ompl[1]
            Angle1 = state_ompl[2]
            Angle2 = state_ompl[3]
            return [Vec_X, Vec_Y, Angle1, Angle2]

    def conver_list_config_to_ompl_config(self, state_list):
        if self.configure_type == "Rigidbody_2D":
            X = state_list[0]
            Y = state_list[1]
            Yaw = state_list[2]

            state_ompl = ob.State(self.space)
            state_ompl().setX(X)
            state_ompl().setY(Y)
            state_ompl().setYaw(Yaw)
            return state_ompl

        if self.configure_type == "Two_Link_2D":
            Vec_X = state_list[0]
            Vec_Y = state_list[1]
            Angle1 = state_list[2]
            Angle2 = state_list[3]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = Vec_X
            state_ompl()[0][1] = Vec_Y
            state_ompl()[1].value = Angle1
            state_ompl()[2].value = Angle2
            return state_ompl

    def solve_planning_2D(self, start, goal, time_lim=10, simple=False):
        self.ss.setStartAndGoalStates(start, goal)
        solved = self.ss.solve(time_lim)
        path = []
        if solved:
            # try to shorten the path
            if simple:
                self.ss.simplifySolution()
            # print the simplified path
            # print(self.ss.getSolutionPath())

        # self.ss.getSolutionPath().interpolate(10)
        if solved:
            states = self.ss.getSolutionPath().getStates()
            path_len = len(states)
            path = []
            if self.configure_type == "Rigidbody_2D":
                for i in range(path_len):
                    X = states[i].getX()
                    Y = states[i].getY()
                    Yaw = states[i].getYaw()
                    path.append([X, Y, Yaw])

            if self.configure_type == "Two_Link_2D":
                path_len = len(states)
                for i in range(path_len):
                    Vec = states[i][0]
                    Angle1 = states[i][1]
                    Angle2 = states[i][2]
                    Vec_X = Vec[0]
                    Vec_Y = Vec[1]
                    Angle1_Yaw = Angle1.value
                    Angle2_Yaw = Angle2.value
                    path.append([Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw])
        return solved, path
