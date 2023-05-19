import os
import csv

import numpy as np

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
        
def read_csv(csv_file):
    info = []
    with open(file=csv_file, mode='r', encoding='utf-8', newline='') as fp:
        dictreader = csv.DictReader(fp)
        for r in dictreader:
            info.append(r)
    return info


def write_key(file, dict):
    keys = list(dict.keys())
    with open(file=file, mode='w', encoding='utf-8', newline='') as fp:
        dictWriter = csv.DictWriter(fp, keys)
        dictWriter.writeheader()
        dictWriter.writerow(dict)
    
def change_S2D_cloud_to_PointNet_format():
    cloud_data = np.load("/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000.npy")
    print(cloud_data.shape)
    print(cloud_data[0, :14])
    cloud_data_1 = cloud_data.reshape(2000, 1400, 2)
    cloud_data_2 = np.transpose(cloud_data_1, (0, 2, 1))
    print(cloud_data_2.shape)
    print(cloud_data_2[0, 0, :7])
    print(cloud_data_2[0, 1, :7])
    np.save("/home/wyhboos/Project/MPMDN/Data/S2D/obs_cloud_2000_PointNet.npy", cloud_data_2)

    
    
    
if __name__ == '__main__':
    change_S2D_cloud_to_PointNet_format()
