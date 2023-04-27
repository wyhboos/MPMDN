#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
import os
import time
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
import sys
sys.path.append("./build")
from MPN import *
er = Env_Robot()


# print(dir(ob.SpaceInformation))
# print(dir(og))
class Plan_OMPL:
    def __init__(self, configure_type="Rigidbody_2D", set_bounds=(-15, 15)):

        self.planner = None
        self.StateValidityChecker = None
        self.configure_type = configure_type
        
        print("Plan_OMPL", configure_type, set_bounds)

        if configure_type == "Rigidbody_2D":
            # create an SE2 state space
            self.space = ob.SE2StateSpace()
            # set bounds
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(set_bounds[0])
            bounds.setHigh(set_bounds[1])
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
            bounds.setLow(set_bounds[0])
            bounds.setHigh(set_bounds[1])
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

        if configure_type == "Three_Link_2D":
            # state space components
            vector_space = ob.RealVectorStateSpace(2)  # 2D vector space
            angle_space1 = ob.SO2StateSpace()  # 1D angle space 1
            angle_space2 = ob.SO2StateSpace()  # 1D angle space 2
            angle_space3 = ob.SO2StateSpace()  # 1D angle space 2
            bounds = ob.RealVectorBounds(2)  # set bounds on vector
            bounds.setLow(set_bounds[0])
            bounds.setHigh(set_bounds[1])
            vector_space.setBounds(bounds)
            # create space
            self.space = ob.CompoundStateSpace()
            # weight 1.0 as default, set for computing distance
            self.space.addSubspace(vector_space, 1.0)
            self.space.addSubspace(angle_space1, 0.5)
            self.space.addSubspace(angle_space2, 0.5)
            self.space.addSubspace(angle_space3, 0.5)

            # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

    def setStateValidityChecker(self, StateValidityChecker):
        self.StateValidityChecker = StateValidityChecker
        self.ss.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.StateValidityChecker))
        

    def set_planner(self, planner="MPMDN"):
        if planner == "MPN":
            self.planner = MPN(self.si)
        if planner == "MPMDN":
            self.planner = MPMDN(self.si)
        elif planner == "RRTstar":
            self.planner = og.RRTstar(self.si)
        elif planner == "RRT":
            # self.planner = og.InformedRRTstar(self.si)
            self.planner = og.RRT(self.si)
        elif planner == "BITstar":
            self.planner = og.BITstar(self.si)
        elif planner == "IRRTstar":
            self.planner = og.InformedRRTstar(self.si)
            # self.planner.setRange(3)
            # self.planner.setGoalBias(0.01)
        self.ss.setPlanner(self.planner)
    
    def set_path_cost_threshold(self, cost=999):
        self.termination = ob.PathLengthOptimizationObjective(self.si)
        self.termination.setCostThreshold(cost)
        self.ss.setOptimizationObjective(self.termination)
        

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
        
        if self.configure_type == "Two_Link_2D":
            Vec_X = state_ompl[0]
            Vec_Y = state_ompl[1]
            Angle1 = state_ompl[2]
            Angle2 = state_ompl[3]
            Angle3 = state_ompl[4]
            return [Vec_X, Vec_Y, Angle1, Angle2, Angle3]

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
        
        if self.configure_type == "Three_Link_2D":
            Vec_X = state_list[0]
            Vec_Y = state_list[1]
            Angle1 = state_list[2]
            Angle2 = state_list[3]
            Angle3 = state_list[4]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = Vec_X
            state_ompl()[0][1] = Vec_Y
            state_ompl()[1].value = Angle1
            state_ompl()[2].value = Angle2
            state_ompl()[3].value = Angle3
            return state_ompl

    def solve_planning_2D(self, start, goal, time_lim=10, simple=False):
        self.ss.clear()
        self.ss.setStartAndGoalStates(start, goal)
        solved = self.ss.solve(time_lim)
        path = []
        if solved:
            # try to shorten the path
            if simple:
                sp_st = time.time()
                self.ss.simplifySolution()
                sp_et = time.time()
                self.sp_ct = sp_et-sp_st
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

            if self.configure_type == "Three_Link_2D":
                path_len = len(states)
                for i in range(path_len):
                    Vec = states[i][0]
                    Angle1 = states[i][1]
                    Angle2 = states[i][2]
                    Angle3 = states[i][3]
                    Vec_X = Vec[0]
                    Vec_Y = Vec[1]
                    Angle1_Yaw = Angle1.value
                    Angle2_Yaw = Angle2.value
                    Angle3_Yaw = Angle3.value
                    path.append([Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw, Angle3_Yaw])
        return solved, path
