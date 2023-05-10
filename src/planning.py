#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

from plan_ompl import Plan_OMPL
from env_robot import Env_Robot
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link, vis_for_2D_planning_three_link

from ompl import base, geometric
import numpy as np
import argparse
import csv

# print(dir(geometric.SimpleSetup))
# print(dir(geometric))


class Plan:
    def __init__(self, type="Two_Link_2D", planner='MPN', set_bounds=(-15, 15)):
        self.type = type
        self.pl_ompl = Plan_OMPL(configure_type=self.type, set_bounds=set_bounds)
        self.env_rob = Env_Robot(robot_type=self.type)

        self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid)
        self.pl_ompl.set_planner(planner)

    # This function aims to solve problems when multiple plan with python using C++ ompl lib
    def reboot(self):
        self.pl_ompl.ss.clear()

    def plan(self, start=None, goal=None, vis=None, time_lim=0.5, simple=False):
        if start is None:
            start, goal = self.pl_ompl.generate_valid_start_goal()
        print("start", start)
        print("goal", goal)
        print("vis", vis)
        solved, path = self.pl_ompl.solve_planning_2D(
            start=start, goal=goal, time_lim=time_lim, simple=simple)
        if vis is not None:
            self.start_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(start)
            self.goal_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(goal)
            self.path_rob_vis = self.env_rob.get_config_path_with_robot_info_2D(path)
            # self.vis(rec_env=self.env_rob.obstacles_vis, start=self.start_vis, goal=self.goal_vis,
            #                              path=self.path_rob_vis, size=50, pixel_per_meter=20, save_fig_dir=vis)
            # print("Fig Saved!")
        return solved, path

    def vis(self, rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
        if self.type == "Two_Link_2D":
            vis_for_2D_planning_two_link(rec_env=rec_env, start=start, goal=goal,
                                         path=path, size=size, pixel_per_meter=pixel_per_meter, save_fig_dir=save_fig_dir)

        if self.type == "Rigidbody_2D" or self.type == "Point_2D":
            vis_for_2D_planning_rigidbody(rec_env=rec_env, start=start, goal=goal,
                                          path=path, size=size, pixel_per_meter=pixel_per_meter, save_fig_dir=save_fig_dir)
            
        if self.type == "Three_Link_2D":
            vis_for_2D_planning_three_link(rec_env=rec_env, start=start, goal=goal,
                                         path=path, size=size, pixel_per_meter=pixel_per_meter, save_fig_dir=save_fig_dir)


