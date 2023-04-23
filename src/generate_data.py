#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
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
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--part', type=int, default=0)
    parser.add_argument('--type', type=int, default="Rigidbody_2D")
    args = parser.parse_args()
    generate_path_main(args)