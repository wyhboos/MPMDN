#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
from utility import *

def get_statistics_classical(para_dict):
    para_index = para_dict["para_index"]
    type = para_dict["type"]
    see = para_dict["see"]
    planner = para_dict["planner"]
    vis_flag = para_dict["vis_flag"]
    ref_file = para_dict["ref_file"]
    use_ref = para_dict["use_ref"]
    simplelify = para_dict["simplelify"]
    mode = "para_"+str(para_index) + "_ref_" + str(int(use_ref)) + "_simp_" + str(int(simplelify))
    
    
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
        model_name = "S2D_TL_" + planner + mode
        vis = "./fig/S2D/ThreeL/"+ planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/S2D/S2D_ThreeL_sg_ev.npy"
        env_file = "./Data/S2D/S2D_env_30000_rec.npy"
    
    if type == "Point_3D":
        model_name = "C3D_Pt_" + planner + mode
        vis = "./fig/C3D/Pt/" + planner +"/" + see + "/" + mode + "/"
        create_dir(vis)
        s_g_file = "./Data/C3D/C3D_Pt_sg_ev.npy"
        env_file = "./Data/C3D/c3d_obs_rec_50000.npy"

    # load env
    rec_envs = np.load(env_file, allow_pickle=True)
    pl = Plan(type, planner, set_bounds=(-20, 20))
    g_s_g = 0

    # generate start and goal
    if g_s_g:
        generate_start_goal(pl=pl,rec_envs=rec_envs,cnt=(1000,10),s_g_file=s_g_file,rm_trivial=True)
        return
        # load start goal
    else:
        env_pts = np.load(s_g_file, allow_pickle=True)
        print("Load start goal suc!")
    
    #read MPN length
    file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_hyrp_0_seen_detail_data.csv"
    dict_list = read_csv(ref_file)
    print(len(dict_list))
    # statistacs
    time_all_all = []
    len_all = []
    node_cnt_all = []
    suc_all = []
        
    datas = []
    datas.append(time_all_all)
    datas.append(len_all)
    datas.append(node_cnt_all)
    datas.append(suc_all)
    
    data = []
        
    # planning
    for i in range(pl_env_s,pl_env_e):
        print("Planning Env:", i)
        rec_env = rec_envs[i, :, :]
        pl.env_rob.load_rec_obs_2D(rec_env)
        for j in range(10):
            # pl.reboot()
            print("Planning Env, path:", i, j)
            start = env_pts[i][j][0]
            goal = env_pts[i][j][1]
            start = pl.pl_ompl.conver_list_config_to_ompl_config(start)
            goal = pl.pl_ompl.conver_list_config_to_ompl_config(goal)
            rec_env = rec_envs[i, :, :]
            pl.pl_ompl.setStateValidityChecker(pl.env_rob.is_state_valid)
            
            #set termination condition 
            if see == "seen":
                mpn_suc = dict_list[int(10*(i)+j)]['suc']
            else:
                mpn_suc = dict_list[int(10*(i-900)+j)]['suc']
            if mpn_suc:
                if see == "seen": 
                    mpn_length = float(dict_list[int(10*(i)+j)]['length'])*1.1
                else:
                    mpn_length = float(dict_list[int(10*(i-900)+j)]['length'])*1.1
            else:
                mpn_length = 9999
                
            if not use_ref:
                mpn_length = 9999
                
            print("mpn_length", mpn_length)
            pl.pl_ompl.set_path_cost_threshold(mpn_length)
                
            solved, path = pl.plan(start=start, goal=goal, vis="yes", time_lim=5, simple=simplelify)
            
            if solved and solved.asString() == "Exact solution":
                suc = 1
            else:
                suc = 0
            
            if simplelify:
                time_all = pl.pl_ompl.ss.getLastPlanComputationTime() + pl.pl_ompl.sp_ct
            else:
                time_all = pl.pl_ompl.ss.getLastPlanComputationTime()
            length = pl.pl_ompl.ss.getSolutionPath().length()
            node_cnt = pl.pl_ompl.ss.getSolutionPath().getStateCount()
            if vis_flag:
                vis_i_j = vis + model_name + "_env_" + str(i) + "_pts_" + str(j)
                vis_i_j_ = vis_i_j
                vis_i_j_ += "_alt_" + str(int(time_all*100)/100)
                vis_i_j_ += "_len_" + str(int(length*100)/100)
                vis_i_j_ += "_node_" + str(node_cnt)
                vis_i_j_ += "_sc_" + str(suc)


                pl.vis(rec_env=pl.env_rob.obstacles_vis, start=pl.start_vis, goal=pl.goal_vis,
                                            path=pl.path_rob_vis, size=50, pixel_per_meter=20, save_fig_dir=vis_i_j_)
            

            data.clear()
            data.append(time_all)
            data.append(length)
            data.append(node_cnt)
            data.append(suc)
            for ii in range(len(datas)):
                datas[ii].append(data[ii])
    
    average = []
    for d in datas:
        avr = np.array(d).mean(axis=0)
        average.append(avr)
    if type == "Rigidbody_2D" or type == "Two_Link_2D_vec" or type == "Three_Link_2D" or type == "Three_Link_2D_vec":
        csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    elif type == "Point_3D":
        csv_file = "./Data/C3D/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_all", "length", "node_cnt", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()
        
    if type == "Rigidbody_2D" or type == "Two_Link_2D_vec" or type == "Three_Link_2D" or type == "Three_Link_2D_vec":
        csv_file = "./Data/S2D/Sta/" + model_name + "_" + see + "_detail_data.csv"
    elif type == "Point_3D":
        csv_file = "./Data/C3D/Sta/" + model_name + "_" + see + "_detail_data.csv"
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
    
def get_statistics_classical_arm(para_dict, state_valid_func, project_path):
    para_index = para_dict["para_index"]
    type = para_dict["type"]
    see = para_dict["see"]
    planner = para_dict["planner"]
    vis_flag = para_dict["vis_flag"]
    ref_file = para_dict["ref_file"]
    use_ref = para_dict["use_ref"]
    simplelify = para_dict["simplelify"]
    mode = "para_"+str(para_index) + "_ref_" + str(int(use_ref)) + "_simp_" + str(int(simplelify))

    if type == "panda_arm":
        model_name = "panda_arm_" + planner + mode
        s_g_file = project_path + "/Data/panda_arm/panda_arm_s_g_env.npy"

    # load env
    # load env
    pl = Plan(type, planner, set_bounds=None, state_valid_func=state_valid_func)
    env_pts = np.load(s_g_file, allow_pickle=True)
    print("Load start goal suc!")

    #read MPN length
    file = "./Data/S2D/Sta/" + "S2D_RB_MPMDN_hyrp_0_seen_detail_data.csv"
    dict_list = read_csv(ref_file)
    print(len(dict_list))
    # statistacs
    time_all_all = []
    len_all = []
    node_cnt_all = []
    suc_all = []
        
    datas = []
    datas.append(time_all_all)
    datas.append(len_all)
    datas.append(node_cnt_all)
    datas.append(suc_all)
    
    data = []
        
    # planning
    for i in range(30):
        print("Planning path:", i)
        start = env_pts[i][0]
        goal = env_pts[i][1]
            
            #set termination condition 
        mpn_suc = dict_list[int(1*(i))]['suc']

        if mpn_suc:
            mpn_length = float(dict_list[int(i)]['length'])*1.05
        else:
            mpn_length = 9999
        if not use_ref:
            mpn_length = 9999
            
        print("mpn_length", mpn_length)
        pl.pl_ompl.set_path_cost_threshold(mpn_length)
            
        solved, path = pl.plan(start=start, goal=goal, vis="yes", time_lim=15, simple=simplelify)
        
        if solved and solved.asString() == "Exact solution":
            suc = 1
        else:
            suc = 0
        
        if simplelify:
            time_all = pl.pl_ompl.ss.getLastPlanComputationTime() + pl.pl_ompl.sp_ct
        else:
            time_all = pl.pl_ompl.ss.getLastPlanComputationTime()
        length = pl.pl_ompl.ss.getSolutionPath().length()
        node_cnt = pl.pl_ompl.ss.getSolutionPath().getStateCount()


        data.clear()
        data.append(time_all)
        data.append(length)
        data.append(node_cnt)
        data.append(suc)
        for ii in range(len(datas)):
            datas[ii].append(data[ii])
    
    average = []
    for d in datas:
        avr = np.array(d).mean(axis=0)
        average.append(avr)
        
        

    csv_file = project_path + "/Data/panda_arm/Sta/" + model_name + "_" + see + "_avg_data.csv"
    header = ["time_all", "length", "node_cnt", "suc"]
    with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(average)
        f.close()
        
    csv_file = project_path + "/Data/panda_arm/Sta/" + model_name + "_" + see + "_detail_data.csv"
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
if __name__ == '__main__':
    all_dict = {}
    #S2D RB BITstar
        #SEEN
    dict_0 = {"para_index":0,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_5_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_1 = {"para_index":1,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_5_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_2 = {"para_index":2,"type":"Rigidbody_2D", "see":"seen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_5_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        #UNSEEN
    dict_3 = {"para_index":3,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_7_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_4 = {"para_index":4,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_7_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_5 = {"para_index":5,"type":"Rigidbody_2D", "see":"unseen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_RB_MPMDNpara_7_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    #S2D TL RRTstar
        #SEEN
    dict_10 = {"para_index":10,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_11 = {"para_index":11,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_12 = {"para_index":12,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        #UNSEEN
    dict_13 = {"para_index":13,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_14 = {"para_index":14,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_15 = {"para_index":15,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}

    #S2D TL IRRTstar
        #SEEN
    dict_11 = {"para_index":10,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
              "planner":"IRRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_vec_MPMDNpara_1150_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    # dict_11 = {"para_index":11,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":True,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    # dict_12 = {"para_index":12,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":False,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        #UNSEEN
    dict_13 = {"para_index":13,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
              "planner":"IRRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_vec_MPMDNpara_1190_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    # dict_14 = {"para_index":14,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":True,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    # dict_15 = {"para_index":15,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":False,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    
    #S2D TL BITstar
        #SEEN
    dict_12 = {"para_index":12,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_vec_MPMDNpara_1150_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    # dict_11 = {"para_index":11,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":True,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    # dict_12 = {"para_index":12,"type":"Two_Link_2D_vec", "see":"seen", "vis_flag":False, 
    #           "planner":"RRTstar", "use_ref":False,"simplelify": True,
    #           "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        #UNSEEN
    dict_14 = {"para_index":14,"type":"Two_Link_2D_vec", "see":"unseen", "vis_flag":False, 
              "planner":"BITstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_vec_MPMDNpara_1190_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    
    #S2D ThreeL RRTstar
        #SEEN
    dict_30 = {"para_index":30,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_35_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_31 = {"para_index":31,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_35_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
    
    dict_32 = {"para_index":32,"type":"Three_Link_2D", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_35_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        #UNSEEN
    dict_33 = {"para_index":33,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_37_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_34 = {"para_index":34,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":True,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_37_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    dict_35 = {"para_index":35,"type":"Three_Link_2D", "see":"unseen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":False,"simplelify": True,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_37_ocl_1_vck_0_cck_40_unseen_detail_data.csv"}
    
    all_dict["0"] = dict_0
    all_dict["1"] = dict_1
    all_dict["2"] = dict_2
    all_dict["3"] = dict_3
    all_dict["4"] = dict_4
    all_dict["5"] = dict_5
    
    all_dict["10"] = dict_10
    all_dict["11"] = dict_11
    all_dict["12"] = dict_12
    all_dict["13"] = dict_13
    all_dict["14"] = dict_14
    all_dict["15"] = dict_15
    
    all_dict["30"] = dict_30
    all_dict["31"] = dict_31
    all_dict["32"] = dict_32
    all_dict["33"] = dict_33
    all_dict["34"] = dict_34
    all_dict["35"] = dict_35

    
    parser = argparse.ArgumentParser()
    parser.add_argument('--para_index', type=int, default=0)
    args = parser.parse_args()
    para_index = args.para_index
    para_dict = all_dict[str(para_index)]
    get_statistics_classical(para_dict)
