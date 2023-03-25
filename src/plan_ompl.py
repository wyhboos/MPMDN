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


class Plan_OMPL:
    def __init__(self):

        self.StateValidityChecker = None
        # create an SE2 state space
        self.space = ob.SE2StateSpace()

        # set lower and upper bounds
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0)
        bounds.setHigh(20)
        self.space.setBounds(bounds)

        # create a simple setup object
        self.ss = og.SimpleSetup(self.space)

    def setStateValidityChecker(self, StateValidityChecker):
        self.StateValidityChecker = StateValidityChecker
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.StateValidityChecker))

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
            print(self.ss.getSolutionPath())

            states = self.ss.getSolutionPath().getStates()
            path_len = len(states)
            for i in range(path_len):
                X = states[i].getX()
                Y = states[i].getY()
                Yaw = states[i].getYaw()
                path.append([X, Y, Yaw])
        return solved, path

        # for vis
        path_wit_robot = er.get_config_path_with_robot_info_2D(path)
        fig_file = "test"
        vis_for_2D_planning(rec_env=er.obstacles_vis, start=path_wit_robot[0], goal=path_wit_robot[-1],
                            path=path_wit_robot,
                            size=20, pixel_per_meter=100, save_fig_dir=fig_file)
        print("time3", time.time() - time1)


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
    print("time1", time.time() - time1)
    solved = ss.solve(20)
    print("time2", time.time() - time1)
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
    print("time3", time.time() - time1)


if __name__ == "__main__":
    planWithSimpleSetup()
