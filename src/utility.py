import os
import csv

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