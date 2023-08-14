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
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link, vis_for_3D_planning_point_plotly
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
        
        if configure_type == "Point_2D":
            # create an SE2 state space
            vector_space = ob.RealVectorStateSpace(2)
            # set bounds
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(set_bounds[0])
            bounds.setHigh(set_bounds[1])
            vector_space.setBounds(bounds)
            self.space = ob.CompoundStateSpace()
            self.space.addSubspace(vector_space, 1.0)
            
            # # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

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
            
        if configure_type == "Two_Link_2D_vec":
            vector_space1 = ob.RealVectorStateSpace(4)
            # vector_space2 = ob.RealVectorStateSpace(2)
            # set bounds
            # to avoid fold and enable using BIT* and IRRT*, the angle is bounded in (-0.75pi, 0.75pi)
            bounds1 = ob.RealVectorBounds(4)
            bounds1.setLow(0, set_bounds[0])
            bounds1.setLow(1, set_bounds[0])
            bounds1.setLow(2, -3.14*0.75)
            bounds1.setLow(3, -3.14*0.75)
            
            bounds1.setHigh(0, set_bounds[1])
            bounds1.setHigh(1, set_bounds[1])
            bounds1.setHigh(2, 3.14*0.75)
            bounds1.setHigh(3, 3.14*0.75)

            
            vector_space1.setBounds(bounds1)
            # vector_space2.setBounds(bounds2)
            self.space = ob.CompoundStateSpace()
            self.space.addSubspace(vector_space1, 1.0)
            # self.space.addSubspace(vector_space2, 0.5)

            # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)

        if configure_type == "Three_Link_2D_vec":
            vector_space1 = ob.RealVectorStateSpace(5)
            # vector_space2 = ob.RealVectorStateSpace(3)
            # set bounds
            # to avoid fold and enable using BIT* and IRRT*, the angle is bounded in (-0.75pi, 0.75pi)
            bounds1 = ob.RealVectorBounds(5)
            bounds1.setLow(0, set_bounds[0])
            bounds1.setLow(1, set_bounds[0])
            bounds1.setLow(2, -3.14*0.75)
            bounds1.setLow(3, -3.14*0.75)
            bounds1.setLow(4, -3.14*0.75)
            
            bounds1.setHigh(0, set_bounds[1])
            bounds1.setHigh(1, set_bounds[1])
            bounds1.setHigh(2, 3.14*0.75)
            bounds1.setHigh(3, 3.14*0.75)
            bounds1.setHigh(4, 3.14*0.75)
            
            vector_space1.setBounds(bounds1)
            # vector_space2.setBounds(bounds2)
            self.space = ob.CompoundStateSpace()
            self.space.addSubspace(vector_space1, 1.0)
            # self.space.addSubspace(vector_space2, 0.5)
            
            # # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)
            
        if configure_type == "Point_3D":
            # create an SE2 state space
            vector_space = ob.RealVectorStateSpace(3)
            # set bounds
            bounds = ob.RealVectorBounds(3)
            bounds.setLow(set_bounds[0])
            bounds.setHigh(set_bounds[1])
            vector_space.setBounds(bounds)
            self.space = ob.CompoundStateSpace()
            self.space.addSubspace(vector_space, 1.0)
            
            # # create a simple setup object, note that si is needed when setting planner
            self.si = ob.SpaceInformation(self.space)
            self.ss = og.SimpleSetup(self.si)
            
        if configure_type == "panda_arm":
            vector_space = ob.RealVectorStateSpace(7)
            arm_bounds_low = [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671]
            arm_bounds_high = [2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 3.8223, 2.9671]
            bounds = ob.RealVectorBounds(7)
            for i in range(7):
                bounds.setLow(i, arm_bounds_low[i])    
                bounds.setHigh(i, arm_bounds_high[i])    
            vector_space.setBounds(bounds)
            self.space = ob.CompoundStateSpace()
            self.space.addSubspace(vector_space, 1.0)
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
        elif planner == "RRTConnect":
            self.planner = og.RRTConnect(self.si)
        elif planner == "LazyPRMstar":
            self.planner = og.LazyPRMstar(self.si)
        self.ss.setPlanner(self.planner)
    
    def set_path_cost_threshold(self, cost=999):
        self.termination = ob.PathLengthOptimizationObjective(self.si)
        self.termination.setCostThreshold(cost)
        self.ss.setOptimizationObjective(self.termination)
        

    def generate_valid_start_goal(self, motion_ck_fun = None):
        start = ob.State(self.space)
        while True:
            while True:
                start.random()
                if self.StateValidityChecker(start()):
                    break
            goal = ob.State(self.space)
            while True:
                goal.random()
                if self.StateValidityChecker(goal()):
                    break
            if motion_ck_fun is None:
                break
            elif not motion_ck_fun(start(), goal()):
                break
        return start, goal

    def convert_ompl_config_to_list_config(self, state_ompl):
        if self.configure_type == "Point_2D":
            X = state_ompl[0]
            Y = state_ompl[1]
            return [X, Y, 0]
        
        if self.configure_type == "Rigidbody_2D":
            X = state_ompl().getX()
            Y = state_ompl().getY()
            Yaw = state_ompl().getYaw()
            return [X, Y, Yaw]

        if self.configure_type == "Two_Link_2D" or self.configure_type == "Two_Link_2D_vec":
            Vec_X = state_ompl[0]
            Vec_Y = state_ompl[1]
            Angle1 = state_ompl[2]
            Angle2 = state_ompl[3]
            return [Vec_X, Vec_Y, Angle1, Angle2]
        
        if self.configure_type == "Three_Link_2D" or self.configure_type == "Three_Link_2D_vec":
            Vec_X = state_ompl[0]
            Vec_Y = state_ompl[1]
            Angle1 = state_ompl[2]
            Angle2 = state_ompl[3]
            Angle3 = state_ompl[4]
            return [Vec_X, Vec_Y, Angle1, Angle2, Angle3]
        
        if self.configure_type == "Point_3D":
            X = state_ompl[0]
            Y = state_ompl[1]
            Z = state_ompl[2]
            return [X, Y, Z]
        
        if self.configure_type == "panda_arm":
            return [state_ompl[i] for i in range(7)]

    def conver_list_config_to_ompl_config(self, state_list):
        if self.configure_type == "Point_2D":
            X = state_list[0]
            Y = state_list[1]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = X
            state_ompl()[0][1] = Y
            return state_ompl
        
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
        
        
        if self.configure_type == "Two_Link_2D_vec":
            Vec_X = state_list[0]
            Vec_Y = state_list[1]
            Angle1 = state_list[2]
            Angle2 = state_list[3]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = Vec_X
            state_ompl()[0][1] = Vec_Y
            state_ompl()[0][2] = Angle1
            state_ompl()[0][3] = Angle2
            return state_ompl
        
        if self.configure_type == "Three_Link_2D_vec":
            Vec_X = state_list[0]
            Vec_Y = state_list[1]
            Angle1 = state_list[2]
            Angle2 = state_list[3]
            Angle3 = state_list[4]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = Vec_X
            state_ompl()[0][1] = Vec_Y
            state_ompl()[0][2] = Angle1
            state_ompl()[0][3] = Angle2
            state_ompl()[0][4] = Angle3
            return state_ompl
        
        if self.configure_type == "Point_3D":
            X = state_list[0]
            Y = state_list[1]
            Z = state_list[2]

            state_ompl = ob.State(self.space)
            state_ompl()[0][0] = X
            state_ompl()[0][1] = Y
            state_ompl()[0][2] = Z
            return state_ompl
        
        if self.configure_type == "panda_arm":
            X = state_list[0]
            Y = state_list[1]
            Z = state_list[2]

            state_ompl = ob.State(self.space)
            for i in range(7):
                state_ompl()[0][i] = state_list[i]
            return state_ompl

    def solve_planning_2D(self, start, goal, time_lim=10, simple=False, interpolate=None):
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
            if interpolate is not None:
                self.ss.getSolutionPath().interpolate(int(interpolate))
            # print the simplified path
            # print(self.ss.getSolutionPath())

        # self.ss.getSolutionPath().interpolate(10)
        if solved:
            states = self.ss.getSolutionPath().getStates()
            path_len = len(states)
            path = []
            if self.configure_type == "Point_2D":
                for i in range(path_len):
                    X = states[i][0][0]
                    Y = states[i][0][1]
                    path.append([X, Y])

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
                    
                    
            if self.configure_type == "Two_Link_2D_vec":
                path_len = len(states)
                for i in range(path_len):
                    Vec_X = states[i][0][0]
                    Vec_Y = states[i][0][1]
                    Angle1_Yaw = states[i][0][2]
                    Angle2_Yaw = states[i][0][3]
                    path.append([Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw])

            if self.configure_type == "Three_Link_2D_vec":
                path_len = len(states)
                for i in range(path_len):
                    Vec_X = states[i][0][0]
                    Vec_Y = states[i][0][1]
                    Angle1_Yaw = states[i][0][2]
                    Angle2_Yaw = states[i][0][3]
                    Angle3_Yaw = states[i][0][4]
                    path.append([Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw, Angle3_Yaw])
                    
            if self.configure_type == "Point_3D":
                for i in range(path_len):
                    X = states[i][0][0]
                    Y = states[i][0][1]
                    Z = states[i][0][2]
                    path.append([X, Y, Z])
                    
            if self.configure_type == "panda_arm":
                for i in range(path_len):
                    path.append([states[i][0][j] for j in range(7)])

        return solved, path
