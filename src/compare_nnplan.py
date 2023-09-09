#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
from utility import *
import os
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
    if "cloud_type" in list(para_dict.keys()):
        cloud_type = para_dict["cloud_type"]
    else:
        cloud_type = "CAE"
    mode = "para_"+str(para_index)+"_ocl_" + str(int(use_orcle)) + "_vck_" + str(valid_ck_cnt) + "_cck_" +str(colli_ck_cnt)
    if see == "seen":
        pl_env_s = 0
        pl_env_e = 900
    elif see == "unseen":
        pl_env_s = 900
        pl_env_e = 1000
        
    if type == "Point_2D":
        model_name = "S2D_RB_" + planner + mode
        vis = "./fig/S2D/Pt/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_Pt_sg_ev.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Rigidbody_2D":
        model_name = "S2D_RB_" + planner + mode
        vis = "./fig/S2D/RB/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_RB_sg_ev.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    if type == "Two_Link_2D":
        model_name = "S2D_TL_" + planner + mode
        vis = "./fig/S2D/TL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_TL_sg_ev.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
        
    if type == "Two_Link_2D_vec":
        model_name = "S2D_TL_vec_" + planner + mode
        vis = "./fig/S2D/TL_vec/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_env_1000_pts_400_Two_Link_vec_sg.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
        
    if type == "Three_Link_2D":
        model_name = "S2D_ThreeL_" + planner + mode
        vis = "./fig/S2D/ThreeL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_ThreeL_sg_ev_not_trivial.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
        
    if type == "Point_3D":
        model_name = "C3D_Pt_" + planner + mode
        vis = "./fig/C3D/Pt/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/C3D/C3D_Pt_sg_ev_not_trivial.npy"
        env_file = "./Data/C3D/c3d_obs_rec_50000.npy"
    

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    if gen_s_g:
        pl = Plan(type, planner, set_bounds=(-15,15))
    else:
        pl = Plan(type, planner, set_bounds=(-20,20))
    # generate start and goal
    if gen_s_g:
        generate_start_goal(pl=pl,rec_envs=rec_envs,cnt=(1000,10),s_g_file=s_g_file,rm_trivial=True)
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
    pl.pl_ompl.planner.state_type = type
    pl.pl_ompl.planner.cloud_type = cloud_type
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
        pl.env_rob.load_rec_obs(rec_env)
        pl.pl_ompl.planner.env_index = i
        for j in range(10):
            # pl.reboot()
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            # pl.env_rob.load_rec_obs_2D(rec_env)
            pl.pl_ompl.setStateValidityChecker(pl.env_rob.is_state_valid)
            solved, path = pl.plan(start=start, goal=goal, vis="yes", time_lim=0.5, simple=False, interpolate=None)
            
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
    
    if type == "Rigidbody_2D" or type == "Two_Link_2D" or type == "Three_Link_2D" or type == "Two_Link_2D_vec":
        csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    elif type == "Point_3D":
        csv_file = "./Data/C3D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_o", "time_nnrp", "time_classical","time_simplify", "time_all", "length","node_cnt","forward_ori",
              "forward_nnrep", "invalid_o", "invalid_nnrep", "colli_o", "colli_nnrep", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()

    if type == "Rigidbody_2D" or type == "Two_Link_2D" or type == "Three_Link_2D" or type == "Two_Link_2D_vec":
        csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_detail_data.csv"
    elif type == "Point_3D":
        csv_file = "./Data/C3D/Sta/" + model_name + "_" + see + "_detail_data.csv"
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
    
def get_statistics_arm(para_dict, state_valid_func, project_path):
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

    if type == "panda_arm":
        model_name = "panda_arm_" + planner + mode
        s_g_file = project_path + "/Data/panda_arm/panda_arm_s_g_env.npy"


    # load env
    pl = Plan(type, planner, set_bounds=None, state_valid_func=state_valid_func)
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


    # set planner parameters
    pl.pl_ompl.planner.state_type = type
    pl.pl_ompl.planner.use_orcle = use_orcle
    pl.pl_ompl.planner.orcle_time_lim = 5
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
    path_all = []
    for i in range(30):
        print("Planning pt:", i)
        start = env_pts[i][0]
        goal = env_pts[i][1]
        solved, path = pl.plan(start=start, goal=goal, vis=None, time_lim=0.5, simple=False)


        if solved and solved.asString() == "Exact solution":
            suc = 1
            path_all.append(path)
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
    
    csv_file = project_path + "/Data/panda_arm/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_o", "time_nnrp", "time_classical","time_simplify", "time_all", "length","node_cnt","forward_ori",
              "forward_nnrep", "invalid_o", "invalid_nnrep", "colli_o", "colli_nnrep", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()


    csv_file = project_path + "/Data/panda_arm/Sta/" + model_name + "_" + see + "_detail_data.csv"
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
    np.save(project_path + "/Data/panda_arm/Sta/path_all.npy", np.array(path_all, dtype='object'))
    print("Finish")
if __name__ == '__main__':
    all_dict = {}
    # S2D RB
        # MPN SEEN
    dict_0 = {"para_index":0,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_RB_2_ckp_460_libtorch.pt"}
    
    dict_1 = {"para_index":1,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_RB_2_ckp_460_libtorch.pt"}
        # MPN UNSEEN
    dict_2 = {"para_index":2,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_RB_2_ckp_460_libtorch.pt"}
    
    dict_3 = {"para_index":3,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_RB_2_ckp_460_libtorch.pt"}
        # MPMDN SEEN
    dict_4 = {"para_index":4,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_RB_2_ckp_1000_libtorch.pt"}
    
    dict_5 = {"para_index":5,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_RB_2_ckp_1000_libtorch.pt"}
        # MPMDN UNSEEN
    dict_6 = {"para_index":6,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_RB_2_ckp_1000_libtorch.pt"}
    
    dict_7 = {"para_index":7,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_RB_2_ckp_1000_libtorch.pt"}
    
    
    # S2D TL
        # MPN SEEN
    dict_10 = {"para_index":10,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    dict_11 = {"para_index":11,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
        # MPN UNSEEN
    dict_12 = {"para_index":12,"type":"Two_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    dict_13 = {"para_index":13,"type":"Two_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
        # MPMDN SEEN
    dict_14 = {"para_index":14,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_15 = {"para_index":15,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
        # MPMDN UNSEEN
    dict_16 = {"para_index":16,"type":"Two_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_17 = {"para_index":17,"type":"Two_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    # S2D ThreeL
        # MPN SEEN
    dict_30 = {"para_index":30,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    dict_31 = {"para_index":31,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
        # MPN UNSEEN
    dict_32 = {"para_index":32,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":True,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    dict_33 = {"para_index":33,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    dict_320 = {"para_index":320,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_ThreeL_Joint_1_ckp_50_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_ThreeL_Joint_1_ckp_50_Pnet_libtorch.pt"}
    
    dict_330 = {"para_index":330,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_ThreeL_Joint_1_ckp_50_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_ThreeL_Joint_1_ckp_50_Pnet_libtorch.pt"}
    
        # MPMDN SEEN
    dict_34 = {"para_index":34,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    dict_35 = {"para_index":35,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
        # MPMDN UNSEEN
    dict_36 = {"para_index":36,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    dict_37 = {"para_index":37,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    dict_360 = {"para_index":360,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_ThreeL_Joint_1_ckp_56_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_ThreeL_Joint_1_ckp_56_Pnet_libtorch.pt"}
    
    dict_370 = {"para_index":370,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_ThreeL_Joint_1_ckp_56_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_ThreeL_Joint_1_ckp_56_Pnet_libtorch.pt"}
    
    # C3D Point
        # MPN SEEN
    dict_40 = {"para_index":40,"type":"Point_3D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    dict_41 = {"para_index":41,"type":"Point_3D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
        # MPN UNSEEN
    dict_42 = {"para_index":42,"type":"Point_3D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    dict_43 = {"para_index":43,"type":"Point_3D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
        # MPMDN SEEN
    dict_44 = {"para_index":44,"type":"Point_3D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    dict_45 = {"para_index":45,"type":"Point_3D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000_3_2000_rd.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/C3D/Model_structure/MDN_C3D_Point_Joint_1_ckp_298_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/C3D/Model_structure/MDN_C3D_Point_Joint_1_ckp_298_Pnet_libtorch.pt"}
        # MPMDN UNSEEN
    dict_46 = {"para_index":46,"type":"Point_3D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    dict_47 = {"para_index":47,"type":"Point_3D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/C3D/c3d_obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S3D/Model_structure/MDN_S2D_ThreeL_1_ckp_3200_libtorch.pt"}
    
    # # S2D Pt
    #     # MPN SEEN
    # dict_20 = {"para_index":20,"type":"Point_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":0, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":1,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_Pt_libtorch.pt"}
    
    # dict_30 = {"para_index":20,"type":"Three_Link_2D", "see":"seen", "vis_flag":True, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_ThreeL_1_ckp_380_libtorch.pt"}
    
    
    # dict_11 = {"para_index":11,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":10, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    # dict_12 = {"para_index":12,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":20, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    # dict_13 = {"para_index":13,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    # dict_14 = {"para_index":14,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":60, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    # dict_15 = {"para_index":15,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":80, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    # dict_16 = {"para_index":16,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
    #           "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":100, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
    #           "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
    #           "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
    #           "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_Pnet_S2D_RB.pt"}
    
    
    dict_100 = {"para_index":100,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":1, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    dict_101 = {"para_index":101,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
            "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
            "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
            "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
            "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    dict_102 = {"para_index":102,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    dict_103 = {"para_index":103,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":80, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TL_2_ckp_380_libtorch.pt"}
    
    
    dict_110 = {"para_index":110,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":1, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_111 = {"para_index":111,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
            "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
            "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
            "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
            "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_112 = {"para_index":112,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":20, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_113 = {"para_index":113,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    
    dict_114 = {"para_index":114,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":60, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_115 = {"para_index":115,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":80, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    dict_116 = {"para_index":116,"type":"Two_Link_2D", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
        "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":100, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,
        "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy",
        "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/Encoder_S2D.pt",
        "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TL_2_ckp_1000_libtorch.pt"}
    
    
    
    
    # S2D TL vec
    
        # MPN SEEN
    
    dict_1120 = {"para_index":1120,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Pnet_libtorch.pt"}
    
    dict_1130 = {"para_index":1130,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Pnet_libtorch.pt"}
    
        # MPMDN SEEN
    
    dict_1140 = {"para_index":1140,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Pnet_libtorch.pt"}
    
    dict_1150 = {"para_index":1150,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Pnet_libtorch.pt"}   
           
        # MPN UNSEEN
    
    dict_1160 = {"para_index":1160,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Pnet_libtorch.pt"}
    
    dict_1170 = {"para_index":1170,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MPN_S2D_TwoL_vec_Joint_1_train_mode_ckp_240_Pnet_libtorch.pt"}
    
        # MPMDN UNSEEN
    
    dict_1180 = {"para_index":1180,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":False, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Pnet_libtorch.pt"}
    
    dict_1190 = {"para_index":1190,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, "save_inva_colli_pair":False, "gen_s_g":False,
              "planner":"MPMDN", "valid_ck_cnt":0, "colli_ck_cnt":40, "use_orcle":True, "ori_simplify":True, "nn_rep_cnt_lim":0, "iter_cnt_lim":20,"cloud_type":"PointNet",
              "ompl_env_file":"/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy",
              "ompl_Enet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Enet_libtorch.pt",
              "ompl_Pnet_file":"/home/wyhboos/Project/MPMDN/Data/S2D/Model_structure/MDN_S2D_TwoL_vec_Joint_1_ckp_160_Pnet_libtorch.pt"}
    
    
    all_dict["45"] = dict_45
    
    all_dict["1120"] = dict_1120
    all_dict["1130"] = dict_1130
    all_dict["1140"] = dict_1140
    all_dict["1150"] = dict_1150
    
    all_dict["1160"] = dict_1160
    all_dict["1170"] = dict_1170
    all_dict["1180"] = dict_1180
    all_dict["1190"] = dict_1190
    

    
    all_dict["0"] = dict_0
    all_dict["1"] = dict_1
    all_dict["2"] = dict_2
    all_dict["3"] = dict_3
    all_dict["4"] = dict_4
    all_dict["5"] = dict_5
    all_dict["6"] = dict_6
    all_dict["7"] = dict_7
    
    all_dict["10"] = dict_10
    all_dict["11"] = dict_11
    all_dict["12"] = dict_12
    all_dict["13"] = dict_13
    all_dict["14"] = dict_14
    all_dict["15"] = dict_15
    all_dict["16"] = dict_16
    all_dict["17"] = dict_17
    
    all_dict["30"] = dict_30
    all_dict["31"] = dict_31
    all_dict["32"] = dict_32
    all_dict["33"] = dict_33
    all_dict["34"] = dict_34
    all_dict["35"] = dict_35
    all_dict["36"] = dict_36
    all_dict["37"] = dict_37
    
    all_dict["320"] = dict_320
    all_dict["330"] = dict_330
    all_dict["360"] = dict_360
    all_dict["370"] = dict_370
    
    # all_dict["20"] = dict_20
    
    # all_dict["30"] = dict_30
    
    
    
    all_dict["100"] = dict_100
    all_dict["101"] = dict_101
    all_dict["102"] = dict_102
    all_dict["103"] = dict_103
    all_dict["110"] = dict_110
    all_dict["111"] = dict_111
    all_dict["112"] = dict_112
    all_dict["113"] = dict_113
    all_dict["114"] = dict_114
    all_dict["115"] = dict_115
    all_dict["116"] = dict_116




    
    parser = argparse.ArgumentParser()
    parser.add_argument('--para_index', type=int, default=0)
    args = parser.parse_args()
    para_index = args.para_index
    para_dict = all_dict[str(para_index)]
    param_file = "./Data/S2D/Sta/param_" + str(para_index) + ".csv"
    write_key(param_file, para_dict)
    
    # para_dict = all_dict['30']
    
    get_statistics(para_dict)
