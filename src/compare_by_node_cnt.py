#!/usr/bin/env python3
# -*- encoding:utf-8 -*-
from planning import *
from utility import *
def compare_by_node_cnt(param_dict):
    node_file = param_dict["node_file"]
    origin_file = param_dict["origin_file"]
    # save_file = param_dict["save_file"]
    lo = len(list(origin_file))
    save_file = origin_file[0:lo-4] + "_node" + ".csv"
    
    node_list = read_csv(node_file)
    origin_list = read_csv(origin_file)
    
    keys = list(origin_list[0].keys())
    # print(keys)
    all_node_cnt = []
    for d in node_list:
        if int(d["node_cnt"]) not in all_node_cnt:
            all_node_cnt.append(int(d["node_cnt"]))
    all_node_cnt = sorted(all_node_cnt)

    save_node_list = []
    each_node_cnt = [0 for i in range(len(all_node_cnt))]
    for i in range(len(all_node_cnt)):
        dict1 = {}
        dict1["rrt_node"] = all_node_cnt[i]
        for key in keys:
            dict1[key] = 0
        save_node_list.append(dict1)
    
    for i in range(len(node_list)):
        node_cnt_i = int(node_list[i]["node_cnt"])
        index_i = all_node_cnt.index(node_cnt_i)
        each_node_cnt[index_i] += 1
        node_i = origin_list[i]
        for key in keys:
            save_node_list[index_i][key] += float(node_i[key])
            
    for i in range(len(all_node_cnt)):
        save_node_list[i]["path_cnt"] = each_node_cnt[i]
        for key in keys:
            save_node_list[i][key] /= each_node_cnt[i]

        
    header = ["rrt_node"]  + ["path_cnt"] + keys
    w_data = []
    with open(file=save_file, mode='w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(save_node_list)):
            w_data.clear()
            for key in header:
                w_data.append(save_node_list[i][key])
            writer.writerow(w_data)
        f.close()


if __name__ == '__main__':
    all_dict = {}
    #S2D RB
        # MPN SEEN
    dict_0 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_1_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPNpara_0_ocl_0_vck_0_cck_40_seen_detail_data.csv",}
    dict_1 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_1_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPNpara_1_ocl_1_vck_0_cck_40_seen_detail_data.csv",}
        # MPN UNSEEN
    dict_2 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_4_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPNpara_2_ocl_0_vck_0_cck_40_unseen_detail_data.csv",}
    dict_3 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_4_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPNpara_3_ocl_1_vck_0_cck_40_unseen_detail_data.csv",}
        # MPMDN SEEN
    dict_4 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_1_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPMDNpara_4_ocl_0_vck_0_cck_40_seen_detail_data.csv",}
    dict_5 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_1_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPMDNpara_5_ocl_1_vck_0_cck_40_seen_detail_data.csv",}
        #MPMDN UNSEEN
    dict_6 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_4_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPMDNpara_6_ocl_0_vck_0_cck_40_unseen_detail_data.csv",}
    dict_7 = {"node_file":"./Data/S2D/Sta/S2D_RB_BITstarpara_4_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_RB_MPMDNpara_7_ocl_1_vck_0_cck_40_unseen_detail_data.csv",}
    
    #S2D TL
        # MPN SEEN
    dict_10 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_11_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPNpara_10_ocl_0_vck_0_cck_40_seen_detail_data.csv",}
    dict_11 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_11_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPNpara_11_ocl_1_vck_0_cck_40_seen_detail_data.csv",}
        # MPN UNSEEN
    dict_12 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_14_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPNpara_12_ocl_0_vck_0_cck_40_unseen_detail_data.csv",}
    dict_13 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_14_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPNpara_13_ocl_1_vck_0_cck_40_unseen_detail_data.csv",}
        # MPMDN SEEN
    dict_14 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_11_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPMDNpara_14_ocl_0_vck_0_cck_40_seen_detail_data.csv",}
    dict_15 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_11_ref_1_simp_1_seen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPMDNpara_15_ocl_1_vck_0_cck_40_seen_detail_data.csv",}
        #MPMDN UNSEEN
    dict_16 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_14_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPMDNpara_16_ocl_0_vck_0_cck_40_unseen_detail_data.csv",}
    dict_17 = {"node_file":"./Data/S2D/Sta/S2D_TL_RRTstarpara_14_ref_1_simp_1_unseen_detail_data.csv", 
              "origin_file":"./Data/S2D/Sta/S2D_TL_MPMDNpara_17_ocl_1_vck_0_cck_40_unseen_detail_data.csv",}
    
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
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--para_index', type=int, default=0)
    args = parser.parse_args()
    para_index = args.para_index
    para_dict = all_dict[str(para_index)]
    compare_by_node_cnt(para_dict)    