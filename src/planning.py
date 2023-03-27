#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

from plan_ompl import Plan_OMPL
from env_robot import Env_Robot
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link

from ompl import base, geometric
# print(dir(geometric.SimpleSetup))
# print(dir(geometric))


class Plan:
    def __init__(self):
        self.pl_ompl = Plan_OMPL(configure_type="Two_Link_2D")
        self.env_rob = Env_Robot(robot_type="Two_Link_2D")

        self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid_2D)
        self.pl_ompl.set_planner()

    def plan(self, start=None, goal=None, vis=None, time_lim=1, simple=False):
        if start is None:
            start, goal = self.pl_ompl.generate_valid_start_goal()
        solved, path = self.pl_ompl.solve_planning_2D(
            start=start, goal=goal, time_lim=time_lim, simple=simple)
        print("Solve:", solved)
        # print("Path", path)

        # visual
        if vis is not None:
            # start = self.env_rob.robot_size + \
            #     [start().getX(), start().getY(), start().getYaw()]
            # goal = self.env_rob.robot_size + \
            #     [goal().getX(), goal().getY(), goal().getYaw()]
            path_rob = self.env_rob.get_config_path_with_robot_info_2D(path)
            start = path_rob[0]
            goal = path_rob[-1]
            print(path_rob)
            vis_for_2D_planning_two_link(rec_env=self.env_rob.obstacles_vis, start=start, goal=goal,
                                         path=path_rob, size=20, pixel_per_meter=50, save_fig_dir=vis)
            print("Fig Saved!")

        # if solved is not None:
        #     states = self.pl_ompl.ss.getSolutionPath()
        #     a = states.getStates()
        #     print(a[0])
        #     print(a[0][0][0])
        #     # print("Found solution with length", path.length())
        #     # path.interpolate(50)
        #     for i in range(len(states)):
        #         state = states[i]
        #         print(state)
        #         print(state.getSubspaceCount())
        #
        #         vec = state.getComponent(0).getValues()
        #         angle1 = state.getComponent(1).value
        #         angle2 = state.getComponent(2).value
        #         print("(", vec[0], ", ", vec[1], ") (", angle1, ", ", angle2, ")")
        # else:
        #     print("No solution found.")


if __name__ == '__main__':
    for i in range(10):
        pl = Plan()
        pl.plan(vis="./fig/two_link"+str(i))
