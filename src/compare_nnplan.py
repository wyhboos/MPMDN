#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
import os
def get_invalid_pair(env_index, list_path, ompl_path, valid_fun):
    l = len(list_path)
    if l<=2:
        return []
    colli_pair = []
    list_goal = list_path[l-1]
    for i in range(1,l-1):
        ompl_state = ompl_path[i]
        if not valid_fun(ompl_state) and valid_fun(ompl_path[i-1]):
            list_start = list_path[i-1]
            colli_pair.append([env_index, list_start, list_goal])
    return colli_pair

def get_colli_pair(env_index, list_path, ompl_path, valid_fun, colli_fun):
    l = len(list_path)
    if l<=2:
        return []
    colli_pair = []
    list_goal = list_path[l-1]
    for i in range(1,l-1):
        if valid_fun(ompl_path[i-1]) and valid_fun(ompl_path[l-1]):
            if not colli_fun(ompl_path[i-1], ompl_path[i]):
                list_start = list_path[i-1]
                colli_pair.append([env_index, list_start, list_goal])
    return colli_pair

def create_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)

def get_statistics(para_dict):
        # dict_1 = {"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        #       "planner":"MPN", "valid_ck_cnt":10, "colli_ck_cnt":10, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20}
    para_index = para_dict["para_index"]
    type = para_dict["type"]
    see = para_dict["see"]
    save_inva_colli_pair = para_dict["save_inva_colli_pair"]
    gen_s_g = para_dict["gen_s_g"]
    planner = para_dict["planner"]
    ompl_env_file = para_dict["ompl_env_file"]
    Pnet_file = para_dict["ompl_Pnet_file"]
    Enet_file = para_dict["ompl_Enet_file"]
    vis_flag = para_dict["vis_flag"]
    valid_ck_cnt = para_dict["valid_ck_cnt"]
    colli_ck_cnt = para_dict["colli_ck_cnt"]
    use_orcle = para_dict["use_orcle"]
    ori_simplify = para_dict["ori_simplify"]
    nn_rep_cnt_lim = para_dict["nn_rep_cnt_lim"]
    iter_cnt_lim = para_dict["iter_cnt_lim"]
    
    
    
    mode = "para_"+str(para_index)+"_ocl_" + str(int(use_orcle)) + "_vck_" + str(valid_ck_cnt) + "_cck_" +str(colli_ck_cnt)


    if see == "seen":
        pl_env_s = 0
        pl_env_e = 90
    elif see == "unseen":
        pl_env_s = 90
        pl_env_e = 100
        
        
    if type == "Rigidbody_2D":
        model_name = "S2D_RB_" + planner + mode
        vis = "./fig/S2D/RB/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_RB_sg_ev.npy"
    if type == "Two_Link_2D":
        model_name = "S2D_TL_" + planner + mode
        vis = "./fig/S2D/TL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_TL_sg_ev.npy"
    env_file = "./Data/S2D/S2D_env_100_rec.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    if gen_s_g:
        pl = Plan(type, planner, set_bounds=(-15,15))
    else:
        pl = Plan(type, planner, set_bounds=(-20,20))

    # generate start and goal
    if gen_s_g:
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
        
    invalid_pair_all = []
    colli_pair_all = []

    # set planner parameters
    pl.pl_ompl.planner.use_orcle = use_orcle
    pl.pl_ompl.planner.ori_simplify = ori_simplify
    pl.pl_ompl.planner.nn_rep_cnt_lim = nn_rep_cnt_lim
    pl.pl_ompl.planner.iter_cnt_lim = iter_cnt_lim
    pl.pl_ompl.planner.valid_ck_cnt = valid_ck_cnt
    pl.pl_ompl.planner.colli_ck_cnt = colli_ck_cnt
    pl.pl_ompl.planner.env_file = ompl_env_file
    pl.pl_ompl.planner.Enet_file = Enet_file
    pl.pl_ompl.planner.Pnet_file = Pnet_file
    pl.pl_ompl.planner.Pnet_train = True
    pl.pl_ompl.planner.reload_env_net()
    # planning
    for i in range(pl_env_s, pl_env_e):
        print("Planning Env:", i)
        rec_env = rec_envs[i, :, :]
        pl.env_rob.load_rec_obs_2D(rec_env)
        pl.pl_ompl.planner.env_index = i
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
            
            # add invalid start and goal
            if save_inva_colli_pair:
                invalid_pair = get_invalid_pair(i, path, pl.pl_ompl.ss.getSolutionPath().getStates(),pl.pl_ompl.StateValidityChecker)
                invalid_pair_all += invalid_pair
                colli_pair = get_colli_pair(i, path, pl.pl_ompl.ss.getSolutionPath().getStates(),pl.pl_ompl.StateValidityChecker, pl.pl_ompl.si.checkMotion)
                colli_pair_all += colli_pair

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
            if vis_flag:
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
                
    #store invalid pair
    if save_inva_colli_pair:
        invalid_pair_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_invalid_pair.npy"
        np.save(invalid_pair_file, np.array(invalid_pair_all,dtype="object"))
        
        colli_pair_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_colli_pair.npy"
        np.save(colli_pair_file, np.array(colli_pair_all,dtype="object"))
    
    average = []
    for d in datas:
        avr = np.array(d).mean(axis=0)
        average.append(avr)
    
    if type == "Rigidbody_2D" or type == "Two_Link_2D":
        csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_o", "time_nnrp", "time_classical","time_simplify", "time_all", "length","node_cnt","forward_ori",
              "forward_nnrep", "invalid_o", "invalid_nnrep", "colli_o", "colli_nnrep", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()

    if type == "Rigidbody_2D" or type == "Two_Link_2D":
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
    
    
    
if __name__ == '__main__':
    all_dict = []
    dict_0 = {"para_index":0,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_1 = {"para_index":1,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":10, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_2 = {"para_index":2,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":20, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_3 = {"para_index":3,"type":"Rigidbody_2D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_4 = {"para_index":4,"type":"Rigidbody_2D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":60, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_5 = {"para_index":5,"type":"Rigidbody_2D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":80, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    dict_6 = {"para_index":6,"type":"Rigidbody_2D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":100, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_110.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    
    
    all_dict.append(dict_0)
    all_dict.append(dict_1)
    all_dict.append(dict_2)
    all_dict.append(dict_3)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--para_index', type=int, default=0)
    args = parser.parse_args()
    para_index = args.para_index
    para_dict = all_dict[para_index]
    get_statistics(para_dict)
