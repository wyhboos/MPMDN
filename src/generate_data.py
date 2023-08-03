#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
from utility import *
def generate_path_main(args):
    # argument
    part = args.part
    type = args.type
    # part = 0
    # type = "Two_Link_2D"
    if type == 0:
        type = "Rigidbody_2D"
    elif type == 1:
        type = "Two_Link_2D"
    elif type == 2:
        type = "Three_Link_2D"
    elif type == 3:
        type = "Point_3D"
    elif type == 4:
        type = "Point_2D"
    print("Part:", part)
    print("type:", type)
    if type == "Point_2D":
        vis = "./fig/S2D/S2D_Point/S2D_Point"
        path_save_file = "./Data/S2D/1000env_400pt/S2D_Point_Path_" + str(part)
        s_g_file = "./Data/S2D/1000env_400pt/S2D_env_100_pts_4000_Point.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Rigidbody_2D":
        vis = "./fig/S2D/S2D_Rigidbody/S2D_Rigidbody"
        path_save_file = "./Data/S2D/1000env_400pt/S2D_Rigidbody_Path_" + str(part)
        s_g_file = "./Data/S2D/1000env_400pt/S2D_env_100_pts_4000_Rigidbody.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Two_Link_2D":
        vis = "./fig/S2D/S2D_Two_Link/S2D_Two_Link"
        path_save_file = "./Data/S2D/1000env_400pt/S2D_Two_Link_Path_"+str(part)
        s_g_file = "./Data/S2D/1000env_400pt/S2D_env_100_pts_4000_Two_Link_Path.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Three_Link_2D":
        vis = "./fig/S2D/S2D_Three_Link/S2D_3L"
        path_save_file = "./Data/S2D/1000env_400pt/S2D_Three_Link_Path_"+str(part)
        s_g_file = "./Data/S2D/1000env_400pt/S2D_env_100_pts_4000_Three_Link_Path.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Point_3D":
        vis = "./fig/C3D/C3D_Point/C3D_Point"
        path_save_file = "./Data/C3D/1000env_400pt/C3D_Point_Path_new"+str(part)
        s_g_file = "./Data/C3D/1000env_400pt/C3D_env_100_pts_4000_Point_Path.npy"
        env_file = "./Data/C3D/c3d_obs_rec_50000.npy"
    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type,"RRTstar", set_bounds=(-20, 20))
    g_s_g = 0

    # generate start and goal
    if g_s_g:
        generate_start_goal(pl=pl,rec_envs=rec_envs,cnt=(1000,400),s_g_file=s_g_file,rm_trivial=True)
        return
        # load start goal
    else:
        env_pts = np.load(s_g_file, allow_pickle=True)
        print("Load start goal suc!")
    # planning
    paths_all = []
    suc_cnt = 0
    for i in range(50 * part, 50 * (part + 1)):
        print("Planning Env:", i)
        paths_env = []
        for j in range(400):
            print("Planning Env Path:", i, j)
            vis_i_j = vis + "_env_" + str(i) + "_pts_" + str(j)
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            pl.env_rob.load_rec_obs(rec_env)
            solved, path = pl.plan(start=start, goal=goal, vis=None, time_lim=0.25, simple=False)
            if solved and solved.asString() == "Exact solution":
                suc_cnt += 1
                paths_env.append(path)
        paths_all.append(paths_env)
    np.save(path_save_file, np.array(paths_all, dtype="object"))
    print("suc_cnt", suc_cnt)

    
if __name__ == '__main__':
    # generate_start_goal_points()
    parser = argparse.ArgumentParser()
    parser.add_argument('--part', type=int, default=0)
    parser.add_argument('--type', type=int, default="2")
    args = parser.parse_args()
    generate_path_main(args)