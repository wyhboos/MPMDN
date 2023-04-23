#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
import os
def get_statistics():
    type = "Rigidbody_2D"
    planner = "MPN"
    # mode = "_nnrp_0_use_orcle"
    # mode = "_no_replan"
    mode = "_nnrp_20"
    mode = "_hyrp_20"
    mode = "_hyrp_0"


    model_name = "S2D_RB_" + planner + mode
    
    see = "unseen"
    if see == "seen":
        pl_env_s = 0
        pl_env_e = 90
    elif see == "unseen":
        pl_env_s = 90
        pl_env_e = 100
        
    if type == 0:
        type = "Rigidbody_2D"
    elif type == 1:
        type = "Two_Link_2D"
        
    if type == "Rigidbody_2D":
        vis = "./fig/S2D/RB/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_RB_sg_ev.npy"
    if type == "Two_Link_2D":
        vis = "./fig/S2D/TL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_TL_sg_ev.npy"
    env_file = "./Data/S2D/S2D_env_100_rec.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type, planner, set_bounds=(-20,20))
    g_s_g = 0

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
    time_simplify_all = []
    time_all_all = []
    length_all = []
    node_cnt_all = []
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
    datas.append(time_simplify_all)
    datas.append(time_all_all)
    datas.append(length_all)
    datas.append(node_cnt_all)
    datas.append(forward_ori_all)
    datas.append(forward_nnrep_all)
    datas.append(invalid_o_all)
    datas.append(invalid_nnrep_all)
    datas.append(colli_o_all)
    datas.append(colli_nnrep_all)
    datas.append(suc_all)
    
    data = []
        
    # planning
    for i in range(pl_env_s, pl_env_e):
        print("Planning Env:", i)
        rec_env = rec_envs[i, :, :]
        pl.env_rob.load_rec_obs_2D(rec_env)
        pl.pl_ompl.planner.env_index = i
        pl.pl_ompl.planner.use_orcle = True
        pl.pl_ompl.planner.ori_simplify = True
        pl.pl_ompl.planner.nn_rep_cnt_lim = 0
        pl.pl_ompl.planner.iter_cnt_lim = 20
        if planner == "MPMDN":
            pl.pl_ompl.planner.valid_ck_cnt = 3
        for j in range(10):
            # pl.reboot()
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            # pl.env_rob.load_rec_obs_2D(rec_env)
            pl.pl_ompl.setStateValidityChecker(pl.env_rob.is_state_valid_2D)
            solved, path = pl.plan(start=start, goal=goal, vis="yes", time_lim=0.5, simple=False)

            if solved and solved.asString() == "Exact solution":
                suc = 1
            else:
                suc = 0

            time_o = pl.pl_ompl.planner.time_o
            time_nnrp = pl.pl_ompl.planner.time_nnrp
            time_classical = pl.pl_ompl.planner.time_classical
            time_simplify = pl.pl_ompl.planner.time_simplify
            time_all = pl.pl_ompl.planner.time_all
            length = pl.pl_ompl.ss.getSolutionPath().length()
            node_cnt = pl.pl_ompl.ss.getSolutionPath().getStateCount()
            forward_ori = pl.pl_ompl.planner.forward_ori
            forward_nnrep = pl.pl_ompl.planner.forward_nnrep
            invalid_o = pl.pl_ompl.planner.invalid_o
            invalid_nnrep = pl.pl_ompl.planner.invalid_nnrep
            colli_o = pl.pl_ompl.planner.colli_o
            colli_nnrep = pl.pl_ompl.planner.colli_nnrep
            vis_i_j = vis + model_name + "_env_" + str(i) + "_pts_" + str(j)
            vis_i_j_ = vis_i_j
            vis_i_j_ += "_alt_" + str(int(time_all*100)/100)
            vis_i_j_ += "_len_" + str(int(length*100)/100)
            vis_i_j_ += "_node_" + str(node_cnt)
            vis_i_j_ += "_ot_" + str(int(time_o*100)/100)
            vis_i_j_ += "_nrpt_" + str(int(time_nnrp*100)/100)
            vis_i_j_ += "_crpt_" + str(int(time_classical*100)/100)
            vis_i_j_ += "_fr_" + str(forward_ori+forward_nnrep)
            vis_i_j_ += "_iva_" + str(invalid_o+invalid_nnrep)
            vis_i_j_ += "_col_" + str(colli_o+colli_nnrep)
            vis_i_j_ += "_sc_" + str(suc)


            pl.vis(rec_env=pl.env_rob.obstacles_vis, start=pl.start_vis, goal=pl.goal_vis,
                                         path=pl.path_rob_vis, size=50, pixel_per_meter=20, save_fig_dir=vis_i_j_)
            

            data.clear()
            data.append(time_o)
            data.append(time_nnrp)
            data.append(time_classical)
            data.append(time_simplify)
            data.append(time_all)
            data.append(length)
            data.append(node_cnt)
            data.append(forward_ori)
            data.append(forward_nnrep)
            data.append(invalid_o)
            data.append(invalid_nnrep)
            data.append(colli_o)
            data.append(colli_nnrep)
            data.append(suc)
            for ii in range(len(datas)):
                datas[ii].append(data[ii])
    
    average = []
    for d in datas:
        avr = np.array(d).mean(axis=0)
        average.append(avr)
    
    csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_o", "time_nnrp", "time_classical","time_simplify", "time_all", "length","node_cnt","forward_ori",
              "forward_nnrep", "invalid_o", "invalid_nnrep", "colli_o", "colli_nnrep", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()

    csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_detail_data.csv"
    header = ["time_o", "time_nnrp", "time_classical", "time_simplify","time_all" ,"length", "node_cnt","forward_ori",
              "forward_nnrep", "invalid_o", "invalid_nnrep", "colli_o", "colli_nnrep", "suc"]
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

    print("Finish")
        
def get_statistics_classical():
    type = "Rigidbody_2D"
    planner = "IRRTstar"
    mode = ""
    model_name = "S2D_RB_" + planner + mode
    
    see = "seen"
    if see == "seen":
        pl_env_s = 0
        pl_env_e = 90
    elif see == "unseen":
        pl_env_s = 90
        pl_env_e = 100

    if type == "Rigidbody_2D":
        vis = "./fig/S2D/RB/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_RB_sg_ev.npy"
    if type == "Two_Link_2D":
        vis = "./fig/S2D/TL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_TL_sg_ev.npy"
    env_file = "./Data/S2D/S2D_env_100_rec.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type, planner)
    g_s_g = 0

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
    
    #read MPN length
    file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_hyrp_0_seen_detail_data.csv"
    dict_list = read_csv(file)
    print(len(dict_list))
    # statistacs
    time_all_all = []
    len_all = []
    node_cnt_all = []
    suc_all = []
        
    datas = []
    # datas.append(time_o_all)
    # datas.append(time_nnrp_all)
    # datas.append(time_classical_all)
    datas.append(time_all_all)
    datas.append(len_all)
    datas.append(node_cnt_all)
    # datas.append(forward_ori_all)
    # datas.append(forward_nnrep_all)
    # datas.append(invalid_o_all)
    # datas.append(invalid_nnrep_all)
    # datas.append(colli_o_all)
    # datas.append(colli_nnrep_all)
    datas.append(suc_all)
    
    data = []
        
    # planning
    for i in range(pl_env_s,pl_env_e):
        print("Planning Env:", i)
        rec_env = rec_envs[i, :, :]
        pl.env_rob.load_rec_obs_2D(rec_env)
        # pl.pl_ompl.planner.env_index = i
        # pl.pl_ompl.planner.use_orcle = True
        # pl.pl_ompl.planner.nn_rep_cnt_lim = 5
        # pl.pl_ompl.planner.iter_cnt_lim = 30
        for j in range(10):
            # pl.reboot()
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            # pl.env_rob.load_rec_obs_2D(rec_env)
            pl.pl_ompl.setStateValidityChecker(pl.env_rob.is_state_valid_2D)
            
            #set termination condition 
            if see == "seen":
                mpn_suc = dict_list[int(10*(i)+j)]['suc']
            else:
                mpn_suc = dict_list[int(10*(i-90)+j)]['suc']
            if mpn_suc:
                if see == "seen": 
                    mpn_length = float(dict_list[int(10*(i)+j)]['length'])*1.05
                else:
                    mpn_length = float(dict_list[int(10*(i-90)+j)]['length'])*1.05
            else:
                mpn_length = 999
            print("mpn_length", mpn_length)
            pl.pl_ompl.set_path_cost_threshold(mpn_length)
                
            solved, path = pl.plan(start=start, goal=goal, vis="yes", time_lim=15, simple=False)
            
            if solved and solved.asString() == "Exact solution":
                suc = 1
            else:
                suc = 0
            
            # time_o = pl.pl_ompl.planner.time_o
            # time_nnrp = pl.pl_ompl.planner.time_nnrp
            # time_classical = pl.pl_ompl.planner.time_classical
            time_all = pl.pl_ompl.ss.getLastPlanComputationTime()
            length = pl.pl_ompl.ss.getSolutionPath().length()
            node_cnt = pl.pl_ompl.ss.getSolutionPath().getStateCount()
            # forward_ori = pl.pl_ompl.planner.forward_ori
            # forward_nnrep = pl.pl_ompl.planner.forward_nnrep
            # invalid_o = pl.pl_ompl.planner.invalid_o
            # invalid_nnrep = pl.pl_ompl.planner.invalid_nnrep
            # colli_o = pl.pl_ompl.planner.colli_o
            # colli_nnrep = pl.pl_ompl.planner.colli_nnrep
            vis_i_j = vis + model_name + "_env_" + str(i) + "_pts_" + str(j)
            vis_i_j_ = vis_i_j
            vis_i_j_ += "_alt_" + str(int(time_all*100)/100)
            vis_i_j_ += "_len_" + str(int(length*100)/100)
            vis_i_j_ += "_node_" + str(node_cnt)
            # vis_i_j_ += "_ot_" + str(int(time_o*100)/100)
            # vis_i_j_ += "_nrpt_" + str(int(time_nnrp*100)/100)
            # vis_i_j_ += "_crpt_" + str(int(time_classical*100)/100)
            # vis_i_j_ += "_fr_" + str(forward_ori+forward_nnrep)
            # vis_i_j_ += "_iva_" + str(invalid_o+invalid_nnrep)
            # vis_i_j_ += "_col_" + str(colli_o+colli_nnrep)
            vis_i_j_ += "_sc_" + str(suc)


            pl.vis(rec_env=pl.env_rob.obstacles_vis, start=pl.start_vis, goal=pl.goal_vis,
                                         path=pl.path_rob_vis, size=50, pixel_per_meter=20, save_fig_dir=vis_i_j_)
            

            data.clear()
            # data.append(time_o)
            # data.append(time_nnrp)
            # data.append(time_classical)
            data.append(time_all)
            data.append(length)
            data.append(node_cnt)
            # data.append(forward_ori)
            # data.append(forward_nnrep)
            # data.append(invalid_o)
            # data.append(invalid_nnrep)
            # data.append(colli_o)
            # data.append(colli_nnrep)
            data.append(suc)
            for ii in range(len(datas)):
                datas[ii].append(data[ii])
    
    average = []
    for d in datas:
        avr = np.array(d).mean(axis=0)
        average.append(avr)
    
    csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_all", "length", "node_cnt", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()

    csv_file = "./Data/S2D/Sta/" + model_name + "_"  + see+ "_detail_data.csv"
    header = ["time_all_avg", "length_avg", "node_cnt", "suc_avg"]
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

    print("Finish")
    
def read_csv(csv_file):
    info = []
    with open(file=csv_file, mode='r', encoding='utf-8', newline='') as fp:
        dictreader = csv.DictReader(fp)
        for r in dictreader:
            info.append(r)
    return info

def create_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)
    
def compare_by_node_cnt():
    node_file = "./Data/S2D/Sta/" + "S2D_RB_IRRTstar_seen_detail_data.csv"
    # mpn_file = "./Data/S2D/Sta/" + "S2D_RB_MPN_hyrp_0_seen_detail_data.csv"
    # mdn_file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_hyrp_0_seen_detail_data.csv"
    mpn_file = "./Data/S2D/Sta/" + "S2D_RB_MPN_no_replan_seen_detail_data.csv"
    mdn_file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_no_replan_seen_detail_data.csv"
    node_list = read_csv(node_file)
    mpn_list = read_csv(mpn_file)
    mdn_list = read_csv(mdn_file)
    print(len(node_list), len(mpn_list), len(mdn_list))
    keys = list(mpn_list[0].keys())
    print(keys)
    all_node_cnt = []
    for d in node_list:
        if int(d["node_cnt"]) not in all_node_cnt:
            all_node_cnt.append(int(d["node_cnt"]))
    all_node_cnt = sorted(all_node_cnt)
    
    mpn_node_list = []
    mdn_node_list = []
    each_node_cnt = [0 for i in range(len(all_node_cnt))]
    for i in range(len(all_node_cnt)):
        dict1 = {}
        dict2 = {}
        dict1["irrt_node"] = all_node_cnt[i]
        dict2["irrt_node"] = all_node_cnt[i]
        for key in keys:
            dict1[key] = 0
            dict2[key] = 0
        mpn_node_list.append(dict1)
        mdn_node_list.append(dict2)
    
    for i in range(len(node_list)):
        print(i)
        node_cnt_i = int(node_list[i]["node_cnt"])
        index_i = all_node_cnt.index(node_cnt_i)
        each_node_cnt[index_i] += 1
        mpn_i = mpn_list[i]
        mdn_i = mdn_list[i]
        for key in keys:
            mpn_node_list[index_i][key] += float(mpn_i[key])
            mdn_node_list[index_i][key] += float(mdn_i[key])
            
    for i in range(len(all_node_cnt)):
        for key in keys:
            mpn_node_list[i][key] /= each_node_cnt[i]
            mdn_node_list[i][key] /= each_node_cnt[i]

        
    csv_file = "./Data/S2D/Sta/" + "S2D_RB_MPN_no_replan_seen_detail_data_node_compare.csv"
    header = ["irrt_node"]  + keys
    w_data = []
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(mpn_node_list)):
            w_data.clear()
            for key in header:
                w_data.append(mpn_node_list[i][key])
            writer.writerow(w_data)
        f.close()
        
    csv_file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_no_replan_seen_detail_data_node_compare.csv"
    header = ["irrt_node"] + keys
    w_data = []
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(mdn_node_list)):
            w_data.clear()
            for key in header:
                w_data.append(mdn_node_list[i][key])
            writer.writerow(w_data)
        f.close()


if __name__ == '__main__':
    # get_statistics()
    # get_statistics_classical()
    compare_by_node_cnt()