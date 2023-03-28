#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

from plan_ompl import Plan_OMPL
from env_robot import Env_Robot
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link

from ompl import base, geometric
import numpy as np


# print(dir(geometric.SimpleSetup))
# print(dir(geometric))


class Plan:
    def __init__(self):
        self.pl_ompl = Plan_OMPL(configure_type="Two_Link_2D")
        self.env_rob = Env_Robot(robot_type="Two_Link_2D")

        self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid_2D)
        self.pl_ompl.set_planner()

    # This function aims to solve problems when multiple plan with python using C++ ompl lib
    def reboot(self):
        self.pl_ompl.reboot()
        self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid_2D)
        self.pl_ompl.set_planner()

    def plan(self, start=None, goal=None, vis=None, time_lim=0.5, simple=False):
        if start is None:
            start, goal = self.pl_ompl.generate_valid_start_goal()
        solved, path = self.pl_ompl.solve_planning_2D(
            start=start, goal=goal, time_lim=time_lim, simple=simple)
        print("Solve:", solved)

        start_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(start)
        goal_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(goal)
        path_rob_vis = self.env_rob.get_config_path_with_robot_info_2D(path)
        # print(path_rob_vis)
        if vis is not None:
            vis_for_2D_planning_two_link(rec_env=self.env_rob.obstacles_vis, start=start_vis, goal=goal_vis,
                                         path=path_rob_vis, size=30, pixel_per_meter=50, save_fig_dir=vis)
            print("Fig Saved!")

        return solved, path


def generate_path_main():
    # load env
    env_file = ""
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan()

    # generate start and goal
    s_g_file = ""
    env_pts = []
    for i in range(100):
        print("Generating start goal, Env:", i)
        rec_env = rec_envs[i, :, :]
        pl.env_rob.load_rec_obs_2D(rec_env)
        for j in range(4000):
            pts = []
            start, goal = pl.pl_ompl.generate_valid_start_goal()
            start = pl.pl_ompl.convert_ompl_config_to_list_config(start)
            goal = pl.pl_ompl.convert_ompl_config_to_list_config(goal)
            pts.append(start, goal)
        env_pts.append(pts)
    np.save(s_g_file, np.array(env_pts))
    return
    # load start goal
    env_pts = np.load(s_g_file, allow_pickle=True)
    # planning
    vis = ""
    paths_all = []
    path_save_file = "S2D_Two_Link_Path"
    for i in range(100):
        print("Planning Env:", i)
        paths_env = []
        for j in range(4000):
            vis_i_j = vis + "_env_" + str(i) + "_pts_" + str(j)
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            solved, path = pl.plan(start=start, goal=goal, vis=vis_i_j, time_lim=1, simple=False)
            print(solved)
            if solved:
                paths_env.append(path)
        paths_all.append(paths_env)
    np.save(path_save_file, np.array(paths_all))



if __name__ == '__main__':
    generate_path_main()


