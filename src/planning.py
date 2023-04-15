#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

from plan_ompl import Plan_OMPL
from env_robot import Env_Robot
from visualization import vis_for_2D_planning_rigidbody, vis_for_2D_planning_two_link

from ompl import base, geometric
import numpy as np
import argparse
import csv

# print(dir(geometric.SimpleSetup))
# print(dir(geometric))


class Plan:
    def __init__(self, type="Two_Link_2D"):
        self.type = type
        self.pl_ompl = Plan_OMPL(configure_type=self.type)
        self.env_rob = Env_Robot(robot_type=self.type)

        self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid_2D)
        self.pl_ompl.set_planner("MPN")

    # This function aims to solve problems when multiple plan with python using C++ ompl lib
    def reboot(self):
        self.pl_ompl.ss.clear()
        # self.pl_ompl.reboot()
        # self.pl_ompl.setStateValidityChecker(self.env_rob.is_state_valid_2D)
        # self.pl_ompl.set_planner()

    def plan(self, start=None, goal=None, vis=None, time_lim=0.5, simple=False):
        if start is None:
            start, goal = self.pl_ompl.generate_valid_start_goal()
        print("start", start)
        print("goal", goal)
        solved, path = self.pl_ompl.solve_planning_2D(
            start=start, goal=goal, time_lim=time_lim, simple=simple)
        if vis is not None:
            start_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(start)
            goal_vis = self.env_rob.get_list_rec_config_with_robot_from_ompl_state(goal)
            path_rob_vis = self.env_rob.get_config_path_with_robot_info_2D(path)
            self.vis(rec_env=self.env_rob.obstacles_vis, start=start_vis, goal=goal_vis,
                                         path=path_rob_vis, size=50, pixel_per_meter=20, save_fig_dir=vis)
            print("Fig Saved!")
        return solved, path

    def vis(self, rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
        print(self.type)
        if self.type == "Two_Link_2D":
            vis_for_2D_planning_two_link(rec_env=rec_env, start=start, goal=goal,
                                         path=path, size=size, pixel_per_meter=pixel_per_meter, save_fig_dir=save_fig_dir)

        if self.type == "Rigidbody_2D":
            print(6666)
            vis_for_2D_planning_rigidbody(rec_env=rec_env, start=start, goal=goal,
                                          path=path, size=size, pixel_per_meter=pixel_per_meter, save_fig_dir=save_fig_dir)


def generate_path_main(args):
    # argument
    part = args.part
    type = args.type
    if type == 0:
        type = "Rigidbody_2D"
    elif type == 1:
        type = "Two_Link_2D"
    print("Part:", part)
    print("type:", type)
    env_file = "./Data/S2D/S2D_env_100_rec.npy"
    if type == "Rigidbody_2D":
        vis = "./fig/S2D/S2D_Rigidbody/S2D_Rigidbody"
        path_save_file = "./Data/S2D/S2D_Rigidbody_Path_" + str(part)
        s_g_file = "./Data/S2D/S2D_env_100_pts_4000_Rigidbody.npy"
    if type == "Two_Link_2D":
        vis = "./fig/S2D/S2D_Two_Link/S2D_Two_Link"
        path_save_file = "./Data/S2D/S2D_Two_Link_Path_"+str(part)
        s_g_file = "./Data/S2D/S2D_env_100_pts_4000_Two_Link_Path.npy"
    env_file = "./Data/S2D/S2D_env_100_rec.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type)
    g_s_g = 0

    # generate start and goal
    if g_s_g:
        env_pts = []
        for i in range(100):
            print("Generating start goal, Env:", i)
            rec_env = rec_envs[i, :, :]
            pl.env_rob.load_rec_obs_2D(rec_env)
            pts = []
            for j in range(4000):
                print(i, j)
                start, goal = pl.pl_ompl.generate_valid_start_goal()
                start = pl.pl_ompl.convert_ompl_config_to_list_config(start)
                goal = pl.pl_ompl.convert_ompl_config_to_list_config(goal)
                # print(start, goal)
                pts.append([start, goal])
            env_pts.append(pts)
        np.save(s_g_file, np.array(env_pts))
        return
        # load start goal
    else:
        env_pts = np.load(s_g_file, allow_pickle=True)
        print("Load start goal suc!")
    # planning
    paths_all = []
    suc_cnt = 0
    for i in range(10 * part, 10 * (part + 1)):
        print("Planning Env:", i)
        paths_env = []
        for j in range(4000):
            pl.reboot()
            vis_i_j = vis + "_env_" + str(i) + "_pts_" + str(j) + "mpn"
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            pl.env_rob.load_rec_obs_2D(rec_env)
            solved, path = pl.plan(start=start, goal=goal, vis=vis_i_j, time_lim=0.5, simple=False)
            if solved and solved.asString() == "Exact solution":
                suc_cnt += 1
                paths_env.append(path)
        paths_all.append(paths_env)
    # np.save(path_save_file, np.array(paths_all))
    print("suc_cnt", suc_cnt)

def get_statistics(args):
    # argument
    part = args.part
    type = args.type
    part = 0
    type = "Rigidbody_2D"
    model_name = "S2D_RB"
    if type == 0:
        type = "Rigidbody_2D"
    elif type == 1:
        type = "Two_Link_2D"
    print("Part:", part)
    print("type:", type)
    env_file = "./Data/S2D/S2D_env_100_rec.npy"
    if type == "Rigidbody_2D":
        vis = "./fig/S2D/RB/_"
        s_g_file = "./Data/S2D/S2D_RB_sg"
    if type == "Two_Link_2D":
        vis = "./fig/S2D/TL/_"
        s_g_file = "./Data/S2D/S2D_TL_sg"
    env_file = "./Data/S2D/S2D_env_100_rec.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type)
    g_s_g = 1

    # generate start and goal
    if g_s_g:
        env_pts = []
        for i in range(100):
            print("Generating start goal, Env:", i)
            rec_env = rec_envs[i, :, :]
            pl.env_rob.load_rec_obs_2D(rec_env)
            pts = []
            for j in range(10):
                print(i, j)
                start, goal = pl.pl_ompl.generate_valid_start_goal()
                start = pl.pl_ompl.convert_ompl_config_to_list_config(start)
                goal = pl.pl_ompl.convert_ompl_config_to_list_config(goal)
                # print(start, goal)
                pts.append([start, goal])
            env_pts.append(pts)
        np.save(s_g_file, np.array(env_pts))
        print("Generate Start Goal Suc!")
        return
        # load start goal
    else:
        env_pts = np.load(s_g_file, allow_pickle=True)
        print("Load start goal suc!")
    
    # statistacs
    time_o_all = []
    time_nnrp_all = []
    time_classical_all = []
    forward_ori_all = []
    forward_nnrep_all = []
    invalid_o_all = []
    invalid_nnrep_all = []
    colli_o_all = []
    colli_nnrep_all = []
    suc_all = []
        
    datas = []
    datas.append(time_o_all)
    datas.append(time_nnrp_all)
    datas.append(time_classical_all)
    datas.append(forward_ori_all)
    datas.append(forward_nnrep_all)
    datas.append(invalid_o_all)
    datas.append(invalid_nnrep_all)
    datas.append(colli_o_all)
    datas.append(colli_nnrep_all)
    datas.append(suc_all)
    
    data = []
        
    # planning
    for i in range(0, 90):
        print("Planning Env:", i)
        for j in range(10):
            # pl.reboot()
            vis_i_j = vis + "_env_" + str(i) + "_pts_" + str(j) + "mpn"
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            pl.env_rob.load_rec_obs_2D(rec_env)
            solved, path = pl.plan(start=start, goal=goal, vis=vis_i_j, time_lim=0.5, simple=False)
            
            if solved and solved.asString() == "Exact solution":
                suc = 1
            if not solved:
                suc = 0
            
            time_o = pl.pl_ompl.planner.time_o
            time_nnrp = pl.pl_ompl.planner.time_nnrp
            time_classical = pl.pl_ompl.planner.time_classical
            forward_ori = pl.pl_ompl.planner.forward_ori
            forward_nnrep = pl.pl_ompl.planner.forward_nnrep
            invalid_o = pl.pl_ompl.planner.invalid_o
            invalid_nnrep = pl.pl_ompl.planner.invalid_nnrep
            colli_o = pl.pl_ompl.planner.colli_o
            colli_nnrep = pl.pl_ompl.planner.colli_nnrep

            data.clear()
            data.append(time_o)
            data.append(time_nnrp)
            data.append(time_classical)
            data.append(forward_ori)
            data.append(forward_nnrep)
            data.append(invalid_o)
            data.append(invalid_nnrep)
            data.append(colli_o)
            data.append(colli_nnrep)
            data.append(suc)
            for i in range(len(datas)):
                datas[i].append(data[i])
    
    average = []
    for d in datas:
        avr = np.array(d).mean(aixs=0)
        average.append[avr]
    
    csv_file = model_name + "_avg_data.csv"
    header = ["time_o_avg", "time_nnrp_avg", "time_classical_avg", "forward_ori_avg",
              "forward_nnrep_avg", "invalid_o_avg", "invalid_nnrep_avg", "colli_o_avg", "colli_nnrep_avg", "suc_avg"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()

    csv_file = model_name + "_detail_data.csv"
    header = ["time_o_avg", "time_nnrp_avg", "time_classical_avg", "forward_ori_avg",
              "forward_nnrep_avg", "invalid_o_avg", "invalid_nnrep_avg", "colli_o_avg", "colli_nnrep_avg", "suc_avg"]
    w_data = []
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(suc_all)):
            w_data.clear()
            for j in range(len(header)):
                w_data.append(datas[j][i])
            writer.writerow(w_data)
        f.close()

    print("Finish£¡")
        

    

if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--part', type=int, default=0)
    # parser.add_argument('--type', type=int, default="Rigidbody_2D")
    # args = parser.parse_args()
    # generate_path_main(args)
    # vis = "./fig/S2D/S2D_RB/MPN/MPN1_test"
    # pl = Plan(type="Rigidbody_2D")
    # env_file = "./Data/S2D/S2D_env_100_rec.npy"
    # rec_envs = np.load(env_file, allow_pickle=True)
    # rec_env = rec_envs[1, :, :]
    # pl.pl_ompl.planner.env_index = 1
    # pl.env_rob.load_rec_obs_2D(rec_env)
    # for i in range(20):
    #     # pl.reboot()
    #     s,p = pl.plan(vis=vis+str(i))
    #     print(s, p)
    
    get_statistics()
