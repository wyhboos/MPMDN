#!/usr/bin/env python3
# -*- encoding:utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
import time

import sys
import copy
import rospy
import moveit_commander
from shape_msgs.msg import SolidPrimitive
import moveit_msgs
import geometry_msgs
from moveit_msgs.msg import PlanningScene, CollisionObject, RobotTrajectory
import moveit_ros_planning_interface
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse,ApplyPlanningScene,ApplyPlanningSceneRequest,ApplyPlanningSceneResponse
from moveit_msgs.msg import RobotState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
# from geometry_msgs.msg import PoseStamped,Quaternion
# import tf2_ros,tf2_geometry_msgs
import numpy as np

import sys
sys.path.append("/home/wyh/Code/MPMDN/src")
sys.path.append("/home/wyh/Code/MPMDN/build")

sys.path.append("/home/wyhboos/Project/MPMDN/src")
sys.path.append("/home/wyhboos/Project/MPMDN/build")



import csv
from planning import *
# from compare_classical import *
## END_SUB_TUTORIAL

# def transform_eluer_to_quaternion(eluer):
#     quaternion = quaternion_from_euler(eluer[0], eluer[1], eluer[2])
#     return quaternion

    #   tutorial.add_box(location=[0,0,-0.05], name='plane', size = [2,2,0.01])


    #     # for multimodal and environemnt encoding
    #     tutorial.add_box(location=[0.62,0,0.4], name='obs1', size = [0.5,0.55,0.03])
    #     tutorial.add_box(location=[0.62,-0.25,0.2], name='obs2', size = [0.5,0.03,0.4])
    #     tutorial.add_box(location=[0.62,0.25,0.2], name='obs3', size = [0.5,0.03,0.4])

    #     tutorial.add_box(location=[0.27,0,0.2], name='obs4', size = [0.2,1.05,0.03])
    #     tutorial.add_box(location=[0.27,-0.5,0.1], name='obs5', size = [0.2,0.03,0.2])
    #     tutorial.add_box(location=[0.27,0.5,0.1], name='obs6', size = [0.2,0.03,0.2])

    #     tutorial.add_box(location=[0.45,0,0.5], name='obs7', size = [0.05,0.3,0.2])
    #     tutorial.add_box(location=[0.7,0,0.5], name='obs8', size = [0.05,0.3,0.2])

    #     tutorial.add_box(location=[0.27,-0.125,0.275], name='obs_right', size = [0.2,0.02,0.15])

    #     tutorial.add_box(location=[0.27, 0.125,0.29], name='obs_left', size = [0.2,0.02,0.18])

    #     tutorial.add_box(location=[0.27, 0.375,0.26], name='obs_left2', size = [0.2,0.02,0.12])

    #     tutorial.add_box(location=[0.27, -0.375,0.29], name='obs_right2', size = [0.2,0.02,0.18])

    #     tutorial.add_box(location=[0.575, 0,0.425], name='grasp_obj', size = [0.05,0.05,0.05])
    #     tutorial.init_planning_scene_service()

def generate_random_box():
    # 8 regions for 1 box each
    x_range = [[-0.4,-0.05],[0.05,0.4]]
    y_range = [[-0.4,-0.05],[0.05,0.4]]
    z_range = [[0.25,0.45],[0.55,0.75]]
    regions = []
    for xr in x_range:
        for yr in y_range:
            for zr in z_range:
                regions.append([xr,yr,zr])
    
    positions = []
    sizes = []
    size = [0.075, 0.075, 0.075]
    for region in regions:
        x = random.uniform(region[0][0], region[0][1])
        y = random.uniform(region[1][0], region[1][1])
        z = random.uniform(region[2][0], region[2][1])
        positions.append([x,y,z])
        sizes.append(size)
    
    return sizes, positions
        
        
        
        

    

def generate_random_table_case():
    # for the high table two obs
    dis = random.uniform(0.2,0.3)
    obs_length_y = random.uniform(0.25, 0.35)
    obs_length_z = random.uniform(0.175,0.225)
    center_x = random.uniform(0.55,0.6)
    center_y = random.uniform(-0.05,0.05)

    position_obs1 = [center_x-0.5*dis, center_y, 0.4+0.5*obs_length_z]
    position_obs2 = [center_x+0.5*dis, center_y, 0.4+0.5*obs_length_z]
    size_obs1 = [0.05, obs_length_y, obs_length_z]
    size_obs2 = [0.05, obs_length_y, obs_length_z]

    # for the low table four dividers
    center_y_delta = [random.uniform(-0.005, 0.005) for i in range(4)]
    center_y = [-0.375, -0.125, 0.125, 0.375]
    obs_length_z = [random.uniform(0.12, 0.18) for i in range(4)]
    position_obs_all = [[0.27, center_y_delta[i]+center_y[i], 0.2+0.5*obs_length_z[i]] for i in range(4)]
    size_obs_all = [[0.2, 0.01, obs_5length_z[i]] for i in range(4)]

    # intergrate
    size = [size_obs1, size_obs2] + size_obs_all
    position = [position_obs1, position_obs2] + position_obs_all
    return size, position

def gen_rand_tbcase_save(env_cnt, save_file):
    envs = []
    env_0_size = [[0.05,0.3,0.2], [0.05,0.3,0.2], [0.2,0.02,0.15], [0.2,0.02,0.18], [0.2,0.02,0.12], [0.2,0.02,0.18]]
    env_0_pose = [[0.45,0,0.5], [0.7,0,0.5], [0.27,-0.125,0.275], [0.27, 0.125,0.29], [0.27, 0.375,0.26], [0.27, -0.375,0.29]]
    envs.append([env_0_size, env_0_pose])
    for i in range(env_cnt):
        size, position = generate_random_table_case()
        envs.append([size,position])
    envs = np.array(envs)
    print(envs.shape, envs)
    np.save(save_file, np.array(envs))

def change_rectange_to_point_clouds(center_pose, plane_size, point_cnt):
    x_range = plane_size[0]
    y_range = plane_size[1]
    z_range = plane_size[2]
    cloud_points = []
    for i in range(point_cnt):
        x = random.uniform(0,x_range)-0.5*x_range
        y = random.uniform(0,y_range)-0.5*y_range
        z = random.uniform(0,z_range)-0.5*z_range
        cloud_points.append([x+center_pose[0], y+center_pose[1], z+center_pose[2]])
    return cloud_points


def change_box_to_point_clouds_surface(pose, size, point_cnt):
    cloud_points = []

    surface_center_poses = []
    for i in range(3):
        s_c1 =copy.copy(pose)
        s_c2 =copy.copy(pose)
        s_c1[i] = s_c1[i] - 0.5*size[i]
        s_c2[i] = s_c2[i] + 0.5*size[i]
        surface_center_poses.append(s_c1)
        surface_center_poses.append(s_c2)

    surface_sizes = []
    for i in range(3):
        s_si1 =copy.copy(size)
        s_si2 =copy.copy(size)
        s_si1[i] = 0
        s_si2[i] = 0
        surface_sizes.append(s_si1)
        surface_sizes.append(s_si2)

    areas = []
    area_all = 0
    for s in surface_sizes:
        a = compute_area_3d_rectangle(s)
        area_all += a
        areas.append(a)
    
    point_cnt_surface = []
    point_cnt_temp = 0
    for a in areas:
        points = int(point_cnt*a/area_all+0.5)
        point_cnt_surface.append(points)
        point_cnt_temp += points
    cnt_dff = point_cnt-point_cnt_temp
    if cnt_dff != 0:
        for i in range(abs(cnt_dff)):
            index = random.randint(0, 5)
            if cnt_dff>0:
                point_cnt_surface[index] += 1
            else:
                point_cnt_surface[index] -= 1
    for i in range(6):
        cloud_point = change_rectange_to_point_clouds(center_pose=surface_center_poses[i], plane_size=surface_sizes[i], point_cnt=point_cnt_surface[i])
        cloud_points += cloud_point
    return cloud_points

def change_box_to_point_clouds(pose, size, point_cnt):
    x_range = size[0]
    y_range = size[1]
    z_range = size[2]

    cloud_points = []
    for i in range(point_cnt):
        x = random.uniform(0,x_range)-0.5*x_range
        y = random.uniform(0,y_range)-0.5*y_range
        z = random.uniform(0,z_range)-0.5*z_range
        cloud_points.append([x+pose[0], y+pose[1], z+pose[2]])
    return cloud_points

def change_sence_to_point_clouds(scene, point_cnt):
    """
    scene=[[size1,size2,..],[pose1,pose2,..]]
    """
    point_all = []
    l = len(scene[0])
    volume_sum = 0
    points_cnt = []
    volume_all =[]
    pt_cnt_tmp = 0
    for i in range(l):
        # volume = compute_box_volume(size=scene[0][i])
        volume = compute_box_surface_area(size=scene[0][i]) #use surface!
        volume_sum+=volume
        volume_all.append(volume)

    for i in range(l):
        pt_cnt = int(point_cnt*volume_all[i]/volume_sum+0.5)
        pt_cnt_tmp += pt_cnt
        points_cnt.append(pt_cnt)
    cnt_dff = point_cnt-pt_cnt_tmp
    if cnt_dff != 0:
        for i in range(abs(cnt_dff)):
            index = random.randint(0, l-1)
            if cnt_dff>0:
                points_cnt[index] += 1
            else:
                points_cnt[index] -= 1
    # print(points_cnt)
    for i in range(l):
        # points_i = change_box_to_point_clouds(size=list(scene[0])[i], pose=list(scene[1])[i], point_cnt=points_cnt[i])
        points_i = change_box_to_point_clouds_surface(size=list(scene[0])[i], pose=list(scene[1])[i], point_cnt=points_cnt[i])
        # print(points_i)
        print(len(points_i))
        point_all += points_i

    print(np.array(point_all).shape)
    return point_all

def get_cloud_points_save(env_file, save_file):
    envs = np.load(env_file, allow_pickle=True)
    cloud_points_all = []
    for i in range(100):
        env_i = list(list(envs)[i])
        cloud_point_i = change_sence_to_point_clouds(scene=env_i, point_cnt=500)
        cloud_points_all.append(cloud_point_i)
    cloud_points_all = np.array(cloud_points_all)
    print(cloud_points_all.shape)
    cloud_points_all = cloud_points_all.reshape(100, 500, 3)
    cloud_points_all = np.transpose(cloud_points_all, (0,2,1))
    print(cloud_points_all.shape)
    np.save(save_file, cloud_points_all)

def compute_area_3d_rectangle(size):
    area = 1
    for i in range(3):
        if size[i] != 0:
            area *= size[i]
    return area


def compute_box_volume(size):
    return float(size[0]*size[1]*size[2])

def compute_box_surface_area(size):
    return 2*(size[0]*size[1]+size[0]*size[2]+size[1]*size[2])


    #   tutorial.add_box(location=[0,0,-0.05], name='plane', size = [2,2,0.01])


    #     # for multimodal and environemnt encoding
    #     tutorial.add_box(location=[0.62,0,0.4], name='obs1', size = [0.5,0.55,0.03])
    #     tutorial.add_box(location=[0.62,-0.25,0.2], name='obs2', size = [0.5,0.03,0.4])
    #     tutorial.add_box(location=[0.62,0.25,0.2], name='obs3', size = [0.5,0.03,0.4])

    #     tutorial.add_box(location=[0.27,0,0.2], name='obs4', size = [0.2,1.05,0.03])
    #     tutorial.add_box(location=[0.27,-0.5,0.1], name='obs5', size = [0.2,0.03,0.2])
    #     tutorial.add_box(location=[0.27,0.5,0.1], name='obs6', size = [0.2,0.03,0.2])

    #     tutorial.add_box(location=[0.45,0,0.5], name='obs7', size = [0.05,0.3,0.2])
    #     tutorial.add_box(location=[0.7,0,0.5], name='obs8', size = [0.05,0.3,0.2])

    #     tutorial.add_box(location=[0.27,-0.125,0.275], name='obs_right', size = [0.2,0.02,0.15])

    #     tutorial.add_box(location=[0.27, 0.125,0.29], name='obs_left', size = [0.2,0.02,0.18])

    #     tutorial.add_box(location=[0.27, 0.375,0.26], name='obs_left2', size = [0.2,0.02,0.12])

    #     tutorial.add_box(location=[0.27, -0.375,0.29], name='obs_right2', size = [0.2,0.02,0.18])

    #     tutorial.add_box(location=[0.575, 0,0.425], name='grasp_obj', size = [0.05,0.05,0.05])
    #     tutorial.init_planning_scene_service()

def generate_random_table_case():
    # for the high table two obs
    dis = random.uniform(0.2,0.3)
    obs_length_y = random.uniform(0.25, 0.35)
    obs_length_z = random.uniform(0.175,0.225)
    center_x = random.uniform(0.55,0.6)
    center_y = random.uniform(-0.05,0.05)

    position_obs1 = [center_x-0.5*dis, center_y, 0.4+0.5*obs_length_z]
    position_obs2 = [center_x+0.5*dis, center_y, 0.4+0.5*obs_length_z]
    size_obs1 = [0.05, obs_length_y, obs_length_z]
    size_obs2 = [0.05, obs_length_y, obs_length_z]

    # for the low table four dividers
    center_y_delta = [random.uniform(-0.005, 0.005) for i in range(4)]
    center_y = [-0.375, -0.125, 0.125, 0.375]
    obs_length_z = [random.uniform(0.12, 0.18) for i in range(4)]
    position_obs_all = [[0.27, center_y_delta[i]+center_y[i], 0.2+0.5*obs_length_z[i]] for i in range(4)]
    size_obs_all = [[0.2, 0.01, obs_length_z[i]] for i in range(4)]

    # intergrate
    size = [size_obs1, size_obs2] + size_obs_all
    position = [position_obs1, position_obs2] + position_obs_all
    return size, position

def generate_random_table_case_new():
    # for the high table two obs
    dis = random.uniform(0.2,0.3)
    obs_length_y = random.uniform(0.25, 0.35)
    obs_length_y = 0.35
    obs_length_z = random.uniform(0.2,0.25)
    center_x = random.uniform(0.6,0.65)
    center_y = 0

    position_obs1 = [center_x-0.5*dis, center_y, 0.4+0.5*obs_length_z]
    position_obs2 = [center_x+0.5*dis, center_y, 0.4+0.5*obs_length_z]
    position_obs3 = [center_x-0.5*dis, center_y, 0.4+0.5*0.05+0.25+0.5*obs_length_z]
    size_obs1 = [0.05, obs_length_y, obs_length_z]
    size_obs2 = [0.05, obs_length_y, obs_length_z]
    size_obs3 = [0.05, obs_length_y, 0.05]

    # for the low table four dividers
    center_y_delta = [random.uniform(-0.005, 0.005) for i in range(2)]
    center_y = [-0.125, 0.125]
    obs_length_z = [random.uniform(0.15, 0.23) for i in range(2)]
    position_obs_all = [[0.32, center_y_delta[i]+center_y[i], 0.2+0.5*obs_length_z[i]] for i in range(2)]
    size_obs_all = [[0.2, 0.01, obs_length_z[i]] for i in range(2)]

    # intergrate
    size = [size_obs1, size_obs2, size_obs3] + size_obs_all
    position = [position_obs1, position_obs2, position_obs3] + position_obs_all
    return size, position

def gen_rand_tbcase_save(env_cnt, save_file):
    envs = []
    env_0_size = [[0.05,0.3,0.2], [0.05,0.3,0.2], [0.2,0.02,0.15], [0.2,0.02,0.18], [0.2,0.02,0.12], [0.2,0.02,0.18]]
    env_0_pose = [[0.45,0,0.5], [0.7,0,0.5], [0.27,-0.125,0.275], [0.27, 0.125,0.29], [0.27, 0.375,0.26], [0.27, -0.375,0.29]]
    envs.append([env_0_size, env_0_pose])
    for i in range(env_cnt):
        size, position = generate_random_table_case()
        envs.append([size,position])
    envs = np.array(envs)
    print(envs.shape, envs)
    np.save(save_file, np.array(envs))

def gen_rand_tbcase_save_new(env_cnt, save_file):
    envs = []
    for i in range(env_cnt):
        size, position = generate_random_table_case_new()
        envs.append([size,position])
    envs = np.array(envs)
    print(envs.shape, envs)
    np.save(save_file, np.array(envs))


def change_rectange_to_point_clouds(center_pose, plane_size, point_cnt):
    x_range = plane_size[0]
    y_range = plane_size[1]
    z_range = plane_size[2]
    cloud_points = []
    for i in range(point_cnt):
        x = random.uniform(0,x_range)-0.5*x_range
        y = random.uniform(0,y_range)-0.5*y_range
        z = random.uniform(0,z_range)-0.5*z_range
        cloud_points.append([x+center_pose[0], y+center_pose[1], z+center_pose[2]])
    return cloud_points


def change_box_to_point_clouds_surface(pose, size, point_cnt):
    cloud_points = []

    surface_center_poses = []
    for i in range(3):
        s_c1 =copy.copy(pose)
        s_c2 =copy.copy(pose)
        s_c1[i] = s_c1[i] - 0.5*size[i]
        s_c2[i] = s_c2[i] + 0.5*size[i]
        surface_center_poses.append(s_c1)
        surface_center_poses.append(s_c2)

    surface_sizes = []
    for i in range(3):
        s_si1 =copy.copy(size)
        s_si2 =copy.copy(size)
        s_si1[i] = 0
        s_si2[i] = 0
        surface_sizes.append(s_si1)
        surface_sizes.append(s_si2)

    areas = []
    area_all = 0
    for s in surface_sizes:
        a = compute_area_3d_rectangle(s)
        area_all += a
        areas.append(a)
    
    point_cnt_surface = []
    point_cnt_temp = 0
    for a in areas:
        points = int(point_cnt*a/area_all+0.5)
        point_cnt_surface.append(points)
        point_cnt_temp += points
    cnt_dff = point_cnt-point_cnt_temp
    if cnt_dff != 0:
        for i in range(abs(cnt_dff)):
            index = random.randint(0, 5)
            if cnt_dff>0:
                point_cnt_surface[index] += 1
            else:
                point_cnt_surface[index] -= 1
    for i in range(6):
        cloud_point = change_rectange_to_point_clouds(center_pose=surface_center_poses[i], plane_size=surface_sizes[i], point_cnt=point_cnt_surface[i])
        cloud_points += cloud_point
    return cloud_points

def change_box_to_point_clouds(pose, size, point_cnt):
    x_range = size[0]
    y_range = size[1]
    z_range = size[2]

    cloud_points = []
    for i in range(point_cnt):
        x = random.uniform(0,x_range)-0.5*x_range
        y = random.uniform(0,y_range)-0.5*y_range
        z = random.uniform(0,z_range)-0.5*z_range
        cloud_points.append([x+pose[0], y+pose[1], z+pose[2]])
    return cloud_points

def change_sence_to_point_clouds(scene, point_cnt):
    """
    scene=[[size1,size2,..],[pose1,pose2,..]]
    """
    point_all = []
    l = len(scene[0])
    volume_sum = 0
    points_cnt = []
    volume_all =[]
    pt_cnt_tmp = 0
    for i in range(l):
        # volume = compute_box_volume(size=scene[0][i])
        volume = compute_box_surface_area(size=scene[0][i]) #use surface!
        volume_sum+=volume
        volume_all.append(volume)

    for i in range(l):
        pt_cnt = int(point_cnt*volume_all[i]/volume_sum+0.5)
        pt_cnt_tmp += pt_cnt
        points_cnt.append(pt_cnt)
    cnt_dff = point_cnt-pt_cnt_tmp
    if cnt_dff != 0:
        for i in range(abs(cnt_dff)):
            index = random.randint(0, l-1)
            if cnt_dff>0:
                points_cnt[index] += 1
            else:
                points_cnt[index] -= 1
    # print(points_cnt)
    for i in range(l):
        # points_i = change_box_to_point_clouds(size=list(scene[0])[i], pose=list(scene[1])[i], point_cnt=points_cnt[i])
        points_i = change_box_to_point_clouds_surface(size=list(scene[0])[i], pose=list(scene[1])[i], point_cnt=points_cnt[i])
        # print(points_i)
        print(len(points_i))
        point_all += points_i

    print(np.array(point_all).shape)
    return point_all

def get_cloud_points_save(env_file, save_file):
    envs = np.load(env_file, allow_pickle=True)
    cloud_points_all = []
    for i in range(100):
        env_i = list(list(envs)[i])
        cloud_point_i = change_sence_to_point_clouds(scene=env_i, point_cnt=500)
        cloud_points_all.append(cloud_point_i)
    cloud_points_all = np.array(cloud_points_all)
    print(cloud_points_all.shape)
    cloud_points_all = cloud_points_all.reshape(100, 500, 3)
    cloud_points_all = np.transpose(cloud_points_all, (0,2,1))
    print(cloud_points_all.shape)
    np.save(save_file, cloud_points_all)

def compute_area_3d_rectangle(size):
    area = 1
    for i in range(3):
        if size[i] != 0:
            area *= size[i]
    return area


def compute_box_volume(size):
    return float(size[0]*size[1]*size[2])

def compute_box_surface_area(size):
    return 2*(size[0]*size[1]+size[0]*size[2]+size[1]*size[2])


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        #init collision check service
        rospy.loginfo("Initializing stateValidity class")
        self.sv_srv = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        rospy.wait_for_service("check_state_validity")
        rospy.loginfo("Reached this point")


        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.pls_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=20)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.envs = None
        self.state = self.move_group.get_current_state()

        self.planner_id = self.move_group.get_planner_id()
        rospy.loginfo("self.planner_id")
        print("self.planner_id:", self.planner_id)
        # self.init_user_planner(planner="BITstar")
        self.init_user_planner(planner="MPN")
        # self.init_user_planner(planner="IRRTstar")
        # self.init_user_planner(planner="TRRT")
        
    def init_fk_service(self):
        self.fk_service = rospy.ServiceProxy('compute_fk', GetPositionFK)
        rospy.wait_for_service('compute_fk')

        
    def init_ik_service(self):
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)
        rospy.wait_for_service('compute_ik')
        # ik_request.ik_request.group_name = "panda_arm"

    def query_inverse_kinematic_service(self, position, orientation):
        # rospy.wait_for_service('/compute_ik')
        # ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        # 创建请求对象
        ik_request = GetPositionIKRequest()
    
        # # 设置运动组名称
        ik_request.ik_request.group_name = "panda_arm"

        ik_request.ik_request.pose_stamped.header.frame_id = "panda_link0"
        ik_request.ik_request.pose_stamped.pose.position.x = position[0]  # 逆运动学目标x坐标
        ik_request.ik_request.pose_stamped.pose.position.y = position[1]  # 逆运动学目标y坐标
        ik_request.ik_request.pose_stamped.pose.position.z = position[2]  # 逆运动学目标z坐标

        # 设置逆运动学目标的姿态四元数
        ik_request.ik_request.pose_stamped.pose.orientation.x = orientation[0]
        ik_request.ik_request.pose_stamped.pose.orientation.y = orientation[1]
        ik_request.ik_request.pose_stamped.pose.orientation.z = orientation[2]
        ik_request.ik_request.pose_stamped.pose.orientation.w = orientation[3]

        response = self.ik_service(ik_request)
        
        # print(response.solution.joint_state.position[0:7])
        return list(response.solution.joint_state.position[0:7])
    
    def query_forward_kinematic_service(self, joint_state):
        fk_request = GetPositionFKRequest()
        
        fk_request.fk_link_names = "panda_arm"
        

    def init_planning_scene_service(self):
        self.apply_planning_quest_srv = rospy.ServiceProxy('apply_planning_scene',ApplyPlanningScene)

    def apply_robot_joint_state(self, joint_value):
        js = joint_value + [0.035, 0.035]
        self.state.joint_state.position = js

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state = self.state
        request = ApplyPlanningSceneRequest()
        request.scene = planning_scene
        response = self.apply_planning_quest_srv(request)

    def apply_ik_solution(self, position, orientation):
        joint_state_value = self.query_inverse_kinematic_service(position=position, orientation=orientation)
        self.apply_robot_joint_state(joint_state_value) 

    def generate_random_pose(self, pose_index=1):
        if pose_index == 0:
            pose_0_x = random.uniform(0.25,0.35)
            pose_0_y = random.uniform(-0.1, 0.1)
            pose_0_z = random.uniform(0.55, 0.6)
            return [pose_0_x,pose_0_y,pose_0_z]
        elif pose_index == 1:
            pose_1_x = random.uniform(0.54,0.61)
            pose_1_y = random.uniform(-0.08, 0.08)
            pose_1_z = 0.6
            return [pose_1_x,pose_1_y,pose_1_z]
        else:
            pose2_y_list = [-0.5,-0.25,0,0.25,0.5]
            pose_2_x = random.uniform(0.25,0.3)
            pose_2_y = random.uniform(-0.01,0.01)+pose2_y_list[int(random.randint(0,4))]
            # pose_2_y = random.uniform(-0.01,0.01)+pose2_y_list[int(pose_index)]
            pose_2_z = 0.35
            return [pose_2_x,pose_2_y,pose_2_z]
    
    def generate_random_pose_new(self, pose_index=1):
        if pose_index == 0:
            pose_0_x = random.uniform(0.25,0.35)
            pose_0_y = 0
            pose_0_z = random.uniform(0.55, 0.6)
            return [pose_0_x,pose_0_y,pose_0_z]
        elif pose_index == 1:
            pose_1_x = random.uniform(0.59,0.66)
            pose_1_y = random.uniform(-0.05, 0.05)
            pose_1_z = random.uniform(0.55, 0.65)
            return [pose_1_x,pose_1_y,pose_1_z]
        else:
            pose2_y_list = [-0.5,-0.25,0,0.25,0.5]
            pose_2_x = random.uniform(0.30,0.35)
            # pose_2_y = random.uniform(-0.01,0.01)+pose2_y_list[int(random.randint(0,4))]
            pose_2_y = random.uniform(-0.01,0.01)+pose2_y_list[int(2)]
            pose_2_z = 0.35
            return [pose_2_x,pose_2_y,pose_2_z]

    def show_random_pose(self, valid_ck=False):
        orientation = [3.141592653589793, -1.7874680211325668e-16, -0.7853981633974488]
        orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        pose_l = []
        index = int(input("Input pose index"))
        for i in range(10):
            while True:
                pose = self.generate_random_pose_new(pose_index=index)
                self.apply_robot_joint_state([0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483])
                rospy.sleep(0.01)
                jv = self.query_inverse_kinematic_service(pose, orientation)
                if not valid_ck:
                    break
                else:
                    if self.state_valid_checker_joint_value(jv):
                        break
            pose_l.append(jv)
        for i in range(10):
            input("press Enter to show pose")
            self.apply_robot_joint_state(pose_l[i])

    def generate_random_valid_pose_joint_state_for_table_case(self, pose_cnt=100, save_file=None):
        # set the initial pose for the inverse kinematic solution to take as reference
        self.apply_robot_joint_state([0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483])
        rospy.sleep(0.01)
        pose_0_l = []
        pose_1_l = []
        pose_2_l = []
        orientation = [3.141592653589793, -1.7874680211325668e-16, -0.7853981633974488]
        orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2])

        cnt = 0
        while True:
            print(0, cnt)
            pose_0 = self.generate_random_pose_new(pose_index=0)
            pose_0_js = self.query_inverse_kinematic_service(pose_0, orientation)
            valid_flag = self.state_valid_checker_joint_value(pose_0_js)
            if valid_flag:
                cnt += 1
                pose_0_l.append([pose_0,pose_0_js])
            if cnt>= pose_cnt:
                break

        cnt = 0
        while True:
            print(1, cnt)
            pose_1 = self.generate_random_pose_new(pose_index=1)
            pose_1_js = self.query_inverse_kinematic_service(pose_1, orientation)
            valid_flag = self.state_valid_checker_joint_value(pose_1_js)
            if valid_flag:
                cnt += 1
                pose_1_l.append([pose_1,pose_1_js])
            if cnt>= pose_cnt:
                break
        
        cnt = 0
        while True:
            print(2, cnt)
            pose_2 = self.generate_random_pose_new(pose_index=2)
            pose_2_js = self.query_inverse_kinematic_service(pose_2, orientation)
            valid_flag = self.state_valid_checker_joint_value(pose_2_js)
            if valid_flag:
                cnt += 1
                pose_2_l.append([pose_2,pose_2_js])
            if cnt>= pose_cnt:
                break
        
        data = [pose_0_l, pose_1_l, pose_2_l]

        if save_file is not None:
            np.save(save_file, np.array(data, dtype="object"))

        return [pose_0_l, pose_1_l, pose_2_l]

    def for_test_plan_table_case_vis(self,pose_file):
        pose_all = np.load(pose_file, allow_pickle=True)
        pose_0 = pose_all[0]
        pose_1 = pose_all[1]
        pose_2 = pose_all[2]
        print(len(pose_0), len(pose_1), len(pose_2))
        for i in range(100):
            config0 = pose_0[i][1]
            config1 = pose_1[i][1]
            config2 = pose_2[i][1]
            input("Press Enter to show trajectory!")
            s,js = self.plan_start_goal_user(start=config0, goal=config1, time_lim=100, interpolate=50)
            self.show_joint_value_path_new(js)

            input("Press Enter to show trajectory!")
            s,js = self.plan_start_goal_user(start=config1, goal=config2, time_lim=100, interpolate=50)
            self.show_joint_value_path_new(js)
         
    def combine_start_goal_for_table_case(self, pose, pose_file=None, save_s_g_file=None):
        if pose_file is not None:
            pose_all = np.load(pose_file, allow_pickle=True)
        else: 
            pose_all = pose
        s_g_all = []
        pose_0 = pose_all[0]
        pose_1 = pose_all[1]
        pose_2 = pose_all[2]
        for i in range(100):
            print(i)
            for j in range(100):
                s_g_all.append([pose_0[i][1], pose_1[j][1]])
        for i in range(100):
            print(i)
            for j in range(100):
                s_g_all.append([pose_1[i][1], pose_2[j][1]])
        s_g_all = np.array(s_g_all)
        print(s_g_all.shape)
        # np.random.shuffle(s_g_all)
        if save_s_g_file is not None:
            np.save(save_s_g_file, s_g_all)
            print("save suc!")
        return s_g_all

    def add_box_scene(self, size, pose, name):
        l = len(size)
        for i in range(l):
            self.add_box(size=size[i], location=pose[i], name=name[i])
        
    def view_random_box(self):
        select_sence = []
        select_cnt = 0
        while True:
            size, pose = generate_random_box()
            name = ['rb1', 'rb2', 'rb3', 'rb4', 'rb5', 'rb6', 'rb7', 'rb8']
            self.add_box_scene(size, pose, name)
            flag = input("Select input 's':")
            if flag == 's':
                print("Select!")
                select_cnt += 1
                select_sence.append([size, pose])
                if select_cnt>= 10:
                    print(len(select_sence))
                    break
            else:
                print("Not select!")
        np.save("/home/wyhboos/data/random_box_scence.npy", np.array(select_sence))
        

    def load_scene(self, file, index):
        print("Load env_index:", index)
        name = ['obs7', 'obs8', 'obs_right2', 'obs_right1', 'obs_left1', 'obs_left2']
        if self.envs is None:
            self.envs = np.load(file, allow_pickle=True)
        env = list(self.envs)[index]
        size = list(list(env)[0])
        pose = list(list(env)[1])
        self.add_box_scene(size=size, pose=pose, name=name)
    
    def gen_pose_start_goal_for_multiple_env(self, env_file, save_s_g_file):
        
        # load env0's start goal
        # s_g_0 = np.load("/home/wyh/data/table_case_s_g_60000.npy", allow_pickle=True)

        pose_all = []
        start_goal_all = []
        # start_goal_all.append(s_g_0)
        for i in range(0, 20):
            print("For env:", i)
            self.load_scene(file=env_file, index=i)
            pose_i = self.generate_random_valid_pose_joint_state_for_table_case(pose_cnt=100)
            pose_all.append(pose_i)
            s_g_i = self.combine_start_goal_for_table_case(pose_i)
            start_goal_all.append(s_g_i)
        start_goal_all = np.array(start_goal_all)
        print(start_goal_all.shape)
        np.save(save_s_g_file, start_goal_all)

    def for_test_show_path_from_start_goal(self, env_file, s_g_file):
        s_g_all = np.load(s_g_file, allow_pickle=True)
        print(s_g_all.shape)
        for i in range(100):
            env_index = int(input("Input env_index:"))
            s_g_index = int(input("Input path_index:"))
            self.load_scene(file=env_file, index=env_index)
            s_g = list(s_g_all)[env_index]
            s,js, sta = self.plan_start_goal_user(start=s_g[s_g_index][0], goal=s_g[s_g_index][1], interpolate=None, time_lim=10)
            print("Suc:", s)
            self.show_joint_value_path_new(joint_value_path=js)
            
    def for_vis_path_with_multiple_start_goal(self,env_file, s_g_file):
        s_g_file = list(np.load(s_g_file))
        while True:
            env_index = int(input("Enter env index:"))
            self.load_scene(file=env_file, index=env_index)
            s_g = list(s_g_file[env_index])
            s_g_index = int(input("Enter start goal index:"))
            s = list(list(s_g[s_g_index])[0])
            g = list(list(s_g[s_g_index])[1])
            print("start:", s, "goal", g)
            # for i in range(3):
            #     self.apply_robot_joint_state(s)
            #     time.sleep(1)
            #     self.apply_robot_joint_state(g)
            #     time.sleep(1)
            s,js, sta = self.plan_start_goal_user(start=s, goal=g, time_lim=5, interpolate=None)
            self.show_joint_value_path_new(js)


        
    def for_vis_path_with_multiple_start_goal(self,env_file, s_g_file):
        s_g_file = list(np.load(s_g_file))
        while True:
            env_index = int(input("Enter env index:"))
            self.load_scene(file=env_file, index=env_index)
            s_g = list(s_g_file[env_index])
            s_g_index = int(input("Enter start goal index:"))
            s = list(list(s_g[s_g_index])[0])
            g = list(list(s_g[s_g_index])[1])
            print("start:", s, "goal", g)
            # for i in range(3):
            #     self.apply_robot_joint_state(s)
            #     time.sleep(1)
            #     self.apply_robot_joint_state(g)
            #     time.sleep(1)
            s,js = self.plan_start_goal_user(start=s, goal=g, time_lim=5, interpolate=None)
            self.show_joint_value_path_new(js)

    def see_left_or_right(self, path_file):
        
        
        path = list(np.load(path, allow_pickle=True))
        l = len(path)
        for i in range(l):
            js = path[-1]
            self.apply_robot_joint_state(js)
            
            
    def compare_nnplan_arm(self, env_file, s_g_file):
        s_g_file = list(np.load(s_g_file))
        sta_all = []
        for i in range(7):
            env_index = i
            self.load_scene(file=env_file, index=env_index)
            s_g = list(s_g_file[env_index])
            for j in range(0, 500, 100):
                print(i, j)
                s_g_index = j
                s = list(list(s_g[s_g_index])[0])
                g = list(list(s_g[s_g_index])[1])
                s, js, sta = self.plan_start_goal_user(start=s, goal=g, time_lim=5, interpolate=None)
                sta_all.append(sta)
        
        l = len(sta_all)
        average_all =[0 for i in range(4)]
        for i in range(l):
            sta_i = sta_all[i]
            for j in range(4):
                average_all[j] += sta_i[j]
        for i in range(4):
            average_all[i] /= l
            
            
        
        csv_file ="/home/wyhboos/Project/MPMDN/Data/panda_arm/mpn_average.csv"
        header = ["time_all", "time_rp", "length","suc"]
        with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(average_all)
            f.close()
        
        csv_file ="/home/wyhboos/Project/MPMDN/Data/panda_arm/mpn_detail.csv"
        header = ["time_all", "time_rp", "length","suc"]
        with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for l in sta_all:
                writer.writerow(l)
            f.close()
            
    def compare_classical_arm(self, env_file, s_g_file, len_ref_file):
        from utility import read_csv
        dict_list = read_csv(len_ref_file)
        
    
        s_g_file = list(np.load(s_g_file))
        sta_all = []
        for i in range(7):
            env_index = i
            self.load_scene(file=env_file, index=env_index)
            s_g = list(s_g_file[env_index])
            for j in range(0, 500, 100):
                print(i, j)
                s_g_index = j
                ref_index = env_index*5 + (s_g_index/100)
                mpn_length = float(dict_list[int(ref_index)]['length'])*1.5
                print("mpn_length", mpn_length)
                self.pl.pl_ompl.set_path_cost_threshold(mpn_length)
                s = list(list(s_g[s_g_index])[0])
                g = list(list(s_g[s_g_index])[1])
                s, js, sta = self.plan_start_goal_user(start=s, goal=g, time_lim=180, interpolate=None)
                sta_all.append(sta)
        
        l = len(sta_all)
        average_all =[0 for i in range(4)]
        for i in range(l):
            sta_i = sta_all[i]
            for j in range(3):
                average_all[j] += sta_i[j]
        for i in range(3):
            average_all[i] /= l
            
        
        csv_file ="/home/wyhboos/Project/MPMDN/Data/panda_arm/bitstar_average.csv"
        header = ["time_all", "time_rp", "length","suc"]
        with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(average_all)
            f.close()
        
        csv_file ="/home/wyhboos/Project/MPMDN/Data/panda_arm/bitstar_detail.csv"
        header = ["time_all", "time_rp", "length","suc"]
        with open(file=csv_file, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for l in sta_all:
                writer.writerow(l)
            f.close()
                    
        

    def getStateValidity(self, robot_state, group_name='panda_arm', constraints=None, print_depth=False):
        """Given a RobotState and a group name and an optional Constraintsreturn the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)

        # if (not result.valid):
        #     contact_depths = []
        #     for i in range(len(result.contacts)):
        #         contact_depths.append(result.contacts[i].depth)

        #     max_depth = max(contact_depths)
        #     if max_depth < 0.0001:
        #         return True
        #     else:
        #         return False 
    
        return result.valid

    def state_valid_checker_ompl(self, ompl_state):
        joint_value = [ompl_state[0][i] for i in range(7)]
        return self.state_valid_checker_joint_value(joint_value)

    def state_valid_checker_joint_value(self, joint_value):
        js = joint_value + [0.035, 0.035]
        self.state.joint_state.position = js
        valid = False
        for i in range(20):
            try:
                valid = self.getStateValidity(self.state)
            except:
                print("Something wrong when calling state valid checker!Retry ",str(i), "/20!")
                continue
            else:
                break
        return valid

    def plan_random_defaut_planner(self):
        state = self.state
        move_group = self.move_group
        while True:
            joint_goal = move_group.get_random_joint_values()
            js = joint_goal + [0.035, 0.035]
 
            state.joint_state.position = js
            flag = self.getStateValidity(state)
            if flag:
                move_group.set_joint_value_target(joint_goal)
                rospy.loginfo("Valid goal! Plan!")
                break
            else:
                rospy.loginfo("Invalid goal!")



        # 开始规划运动路径
        start_t = time.time()
        plan = move_group.plan()
        print(type(plan))
        print(type(plan[0]))
        c_t = time.time()-start_t
        # print("Time:", c_t)
        # print(plan)
        # print("Length:", len(plan[1].joint_trajectory.points))
        # self.show_joint_value_path(p_j)

    def plan_random(self):
        state = self.state
        move_group = self.move_group

        while True:
            # print(s)
            joint_start = move_group.get_random_joint_values()
            joint_goal = move_group.get_random_joint_values()
            js1 = joint_start + [0.035, 0.035]
            js2 = joint_goal + [0.035, 0.035]
            # print(joint_goal)
            # s = move_group.get_current_state()
            # print(s)
            state.joint_state.position = js1
            flag1 = self.getStateValidity(state)
            state.joint_state.position = js2
            flag2 = self.getStateValidity(state)
            print(flag1, flag2)
            if flag1 and flag2:
                # move_group.set_joint_value_target(joint_goal)
                rospy.loginfo("Valid goal! Plan!")
                break
            else:
                rospy.loginfo("Invalid goal!")



        # 开始规划运动路径
        start_t = time.time()
        s,p_j = self.plan_start_goal_user(joint_start, joint_goal, interpolate=100)
        # print(p_j)
        # plan = move_group.plan()
        c_t = time.time()-start_t
        # print("Time:", c_t)
        # print(plan)
        # print("Length:", len(plan[1].joint_trajectory.points))
        self.show_joint_value_path_new(p_j)
        # self.show_joint_value_path(p_j)
        # self.display_trajectory(plan[1])

        # # 执行规划的运动路径
        # move_group.execute(plan)
    
    def generate_valid_start_goal(self):
        while True:
            joint_goal = self.move_group.get_random_joint_values()
            js = joint_goal + [0.035, 0.035]
            self.state.joint_state.position = js
            if self.getStateValidity(self.state):
                start = joint_goal
                break
        
        while True:
            joint_goal = self.move_group.get_random_joint_values()
            js = joint_goal + [0.035, 0.035]
            self.state.joint_state.position = js
            if self.getStateValidity(self.state):
                goal = joint_goal
                break
        
        return start, goal
    
    def generate_valid_start_goal_save(self, cnt,save_file):
        start_goal = []
        for i in range(cnt):
            if i%100 ==0:
                print(i)
            start, goal = self.generate_valid_start_goal()
            start_goal.append([start, goal])
        
        np.save(save_file, np.array(start_goal))

    def plan_start_goal(self, start, goal):
        state = self.state
        move_group = self.move_group
        state.joint_state.position = (start + [0.035, 0.035])
        move_group.set_start_state(state)
        move_group.set_joint_value_target(goal)
        plan = move_group.plan()
        return plan
    
    def init_user_planner(self, planner = "RRTConnect"):
        if planner == "MPN" or planner == "MPMDN":
            self.pl = Plan(type="panda_arm", planner=planner, set_bounds=None, state_valid_func=self.state_valid_checker_ompl)
            self.pl.pl_ompl.planner.state_type = "panda_arm"
            self.pl.pl_ompl.planner.cloud_type = "PointNet_500"
            self.pl.pl_ompl.planner.use_orcle = False
            self.pl.pl_ompl.planner.orcle_time_lim = 120
            self.pl.pl_ompl.planner.ori_simplify = True
            self.pl.pl_ompl.planner.nn_rep_cnt_lim = 0
            self.pl.pl_ompl.planner.iter_cnt_lim = 20
            self.pl.pl_ompl.planner.valid_ck_cnt = 0
            self.pl.pl_ompl.planner.colli_ck_cnt = 40
            self.pl.pl_ompl.planner.env_file = "/home/wyhboos/Project/MPMDN/Data/panda_arm/tb_env_new_clouds_100_3_500_surface.npy"
            # self.pl.pl_ompl.planner.Enet_file = "/home/wyhboos/Project/MPMDN/Data/panda_arm/Model_structure/MDN_ARM_tb_new_tb_Joint_mix_40_batch2048_ckp_630_Enet_libtorch.pt"
            # self.pl.pl_ompl.planner.Pnet_file = "/home/wyhboos/Project/MPMDN/Data/panda_arm/Model_structure/MDN_ARM_tb_new_tb_Joint_mix_40_batch2048_ckp_630_Pnet_libtorch.pt"
            
            self.pl.pl_ompl.planner.Enet_file = "/home/wyhboos/Project/MPMDN/Data/panda_arm/Model_structure/MPN_ARM_tb_new_Joint_batch2048_1_ckp_350_Enet_libtorch.pt"
            self.pl.pl_ompl.planner.Pnet_file = "/home/wyhboos/Project/MPMDN/Data/panda_arm/Model_structure/MPN_ARM_tb_new_Joint_batch2048_1_ckp_350_Pnet_libtorch.pt"
            self.pl.pl_ompl.planner.Pnet_train = True
            self.pl.pl_ompl.planner.reload_env_net()
        else:
            self.pl = Plan(type="panda_arm", planner=planner, set_bounds=None, state_valid_func=self.state_valid_checker_ompl)
        # self.pl = Plan(type="panda_arm", planner=planner, set_bounds=None, state_valid_func=self.state_valid_checker_ompl)
    
    def plan_start_goal_user(self, start, goal, time_lim=10, interpolate=None):
        pl = self.pl
        # pl.pl_ompl.set_path_cost_threshold(100)
        solve, path = pl.plan(start, goal, time_lim=time_lim, interpolate=interpolate)
        if solve and solve.asString() == "Exact solution":
            suc = 1
        else:
            suc = 0
        # time_o = pl.pl_ompl.planner.time_o
        # time_nnrp = pl.pl_ompl.planner.time_nnrp
        # time_classical = pl.pl_ompl.planner.time_classical
        # time_all = pl.pl_ompl.planner.time_all
        # length = pl.pl_ompl.ss.getSolutionPath().length()
        # return solve, path, [time_all, time_nnrp+time_classical, length, suc]
        
        # time_o = pl.pl_ompl.planner.time_o
        # time_nnrp = pl.pl_ompl.planner.time_nnrp
        # time_classical = pl.pl_ompl.planner.time_classical
        time_all = pl.pl_ompl.ss.getLastPlanComputationTime()
        length = pl.pl_ompl.ss.getSolutionPath().length()
        return solve, path, [time_all, length, suc]
        

    def generate_paths(self, s_g_file, path_save_file):
        s_g = np.load(s_g_file)
        l = len(s_g)
        path_all = []
        for i in range(l):
            path = []
            start = list(s_g[i][0])
            goal = list(s_g[i][1])  
            plan = self.plan_start_goal(start, goal)
            path_l = len(plan[1].joint_trajectory.points)
            for j in range(path_l):
                waypoint = list(plan[1].joint_trajectory.points[0].positions)
                path.append(waypoint)
            path_all.append(path)
            
            if i%2000 == 0:
                np.save(path_save_file+"part_"+str(i)+".npy", np.array(path_all, dtype='object'))

        np.save(path_save_file+"part_"+str(i)+".npy", np.array(path_all, dtype='object'))

    def generate_paths_user_plan(self, s_g_file, path_save_file, thread=0, env_index=None):
        print(s_g_file)
        s_g = np.load(s_g_file, allow_pickle=True)
        if env_index is not None:
            s_g = list(s_g)[env_index]
        # s_g = 
        print(s_g)
        l = len(s_g)
        path_all = []
        suc = 0
        suc_all = []
        time_all = []
        length_all = []
        node_cnt_all = []
        for i in range(10000+int(thread%2)*200, 10000+(int(thread%2)+1)*200):
            print("env_index",env_index, "path:", i, "suc", suc)
            path = []
            start = list(s_g[i][0])
            goal = list(s_g[i][1])
            solve, path = self.plan_start_goal_user(start, goal, time_lim=180)   
            time_all.append(self.pl.pl_ompl.ss.getLastPlanComputationTime())
            if solve.asString() == "Exact solution":
                suc += 1
                suc_all.append(1)
                length_all.append(self.pl.pl_ompl.ss.getSolutionPath().length())
                path_all.append(path+[self.pl.pl_ompl.ss.getSolutionPath().length()])
                node_cnt_all.append(self.pl.pl_ompl.ss.getSolutionPath().getStateCount())
                print("Length:", self.pl.pl_ompl.ss.getSolutionPath().length())
            else:
                suc_all.append(0)
                length_all.append('nan')
                node_cnt_all.append('nan')
            
            if i% 10 == 0 or i ==10000+(int(thread%2)+1)*200-1:
                header = ["time", "length","node_cnt", "suc"]
                with open(file="/home/wyh/data/BITstar_table_case_data_thread"+str(thread)+".csv", mode='w', encoding='utf-8', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                    for i in range(len(suc_all)):
                        writer.writerow([time_all[i], length_all[i],node_cnt_all[i],suc_all[i]])
            if (i% 10 == 0 and i>0) or i == 10000+(int(thread%2)+1)*200-1:
                np.save(path_save_file + "_env_" + str(env_index)  + "thread_" + str(thread) +".npy", np.array(path_all, dtype='object'))

        np.save(path_save_file+"thread_"+str(thread)+".npy", np.array(path_all, dtype='object'))

    def compare_classical(self):
        state_valid_func = self.state_valid_checker_ompl
        para_dict = {"para_index":40,"type":"panda_arm", "see":"seen", "vis_flag":False, 
              "planner":"RRTstar", "use_ref":False, "simplelify": False,
              "ref_file":"./Data/S2D/Sta/" + "S2D_TL_MPMDNpara_35_ocl_1_vck_0_cck_40_seen_detail_data.csv"}
        get_statistics_classical_arm(para_dict, state_valid_func)

    def show_joint_value_path_new(self, joint_value_path):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = self.move_group.get_active_joints()

        l = len(joint_value_path)
        state = self.state
        state.joint_state.position = joint_value_path[0] + [0.035, 0.035]
        display_trajectory.trajectory_start = state
        for i in range(l):
            tp = JointTrajectoryPoint()
            tp.positions = joint_value_path[i] + [0.035, 0.035]
            tp.time_from_start = rospy.Duration(i*1)
            rt.joint_trajectory.points.append(tp)
        display_trajectory.trajectory.append(rt)
        self.display_trajectory_publisher.publish(display_trajectory)
            

    
    def show_joint_value_path(self, joint_value_path):
        plans = []
        l = len(joint_value_path)
        for i in range(l-1):
            plan = self.plan_start_goal(joint_value_path[i], joint_value_path[i+1])
            plans.append(plan)
        self.display_trajectory(plans, joint_value_path[0])

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan, start_joint):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        state = self.state
        state.joint_state.position = start_joint + [0.035, 0.035]
        display_trajectory.trajectory_start = state
        for p in plan:
            display_trajectory.trajectory.append(p[1])
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4, location = (0,0,0), name="box", size=[0.075, 0.075, 0.075]):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        # box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = location[0]  # above the panda_hand frame
        box_pose.pose.position.y = location[1]  # above the panda_hand frame
        box_pose.pose.position.z = location[2]  # above the panda_hand frame
        box_name = name
        scene.add_box(box_name, box_pose, size=size)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        obj = CollisionObject()
        obj.id = box_name
        obj.header.frame_id = "panda_link0"
        obj.operation = obj.ADD
        # print(obj)
        self.box_name = box_name
        box = SolidPrimitive()
        box.type = box.BOX
        # print(box.dimensions)
        box.dimensions = [0,0,0]
        box.dimensions[0] = size[0]
        box.dimensions[1] = size[1]
        box.dimensions[2] = size[2]

        box_pose_ = geometry_msgs.msg.Pose()
        box_pose_.position.x = location[0]  # above the panda_hand frame
        box_pose_.position.y = location[1]  # above the panda_hand frame
        box_pose_.position.z = location[2]  # above the panda_hand frame
        obj.primitives.append(box)
        obj.primitive_poses.append(box_pose_)
        pls = PlanningScene()
        pls.is_diff = True
        pls.world.collision_objects.append(obj)
        self.pls_pub.publish(pls)
        import time
        time.sleep(0.05)
        # scene.apply_planning_scene(pls)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def arm_main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        # input(
        #     "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        # )
        tutorial = MoveGroupPythonInterfaceTutorial()

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # tutorial.go_to_joint_state()

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to add a box to the planning scene ...")
        # tutorial.add_box(location=[0.2,0.1,0.5], name='box1')
        # tutorial.add_box(location=[0.3,0.4,0.3], name='box2')

        # tutorial.add_box(location=[-0.1,0.4,0.7], name='box3')
        # tutorial.add_box(location=[-0.3,0.35,0.25], name='box4')

        # tutorial.add_box(location=[0.1,-0.4,0.6], name='box5')
        # tutorial.add_box(location=[0.3,-0.4,0.35], name='box6')

        # tutorial.add_box(location=[-0.3,-0.3,0.65], name='box7')
        # tutorial.add_box(location=[-0.15,-0.4,0.45], name='box8')

        # tutorial.add_box(location=[-0.15,-0.15,0.8], name='box9')
        # tutorial.add_box(location=[0.15,0.15,0.9], name='boX10')
        
        # tutorial.add_box(location=[0,0,-0.05], name='plane', size = [2,2,0.01])


        # for multimodal and environemnt encoding
        # tutorial.add_box(location=[0.67,0,0.4], name='obs1', size = [0.5,0.55,0.03])
        # tutorial.add_box(location=[0.67,-0.25,0.2], name='obs2', size = [0.5,0.03,0.4])
        # tutorial.add_box(location=[0.67,0.25,0.2], name='obs3', size = [0.5,0.03,0.4])

        # tutorial.add_box(location=[0.32,0,0.2], name='obs4', size = [0.2,1.05,0.03])
        # tutorial.add_box(location=[0.32,-0.5,0.1], name='obs5', size = [0.2,0.03,0.2])
        # tutorial.add_box(location=[0.32,0.5,0.1], name='obs6', size = [0.2,0.03,0.2])

        # # tutorial.add_box(location=[0.45,0,0.5], name='obs7', size = [0.05,0.3,0.2])
        # # tutorial.add_box(location=[0.7,0,0.5], name='obs8', size = [0.05,0.3,0.2])

        # # tutorial.add_box(location=[0.27,-0.125,0.275], name='obs_right', size = [0.2,0.02,0.15])

        # # tutorial.add_box(location=[0.27, 0.125,0.29], name='obs_left', size = [0.2,0.02,0.18])

        # # tutorial.add_box(location=[0.27, 0.375,0.26], name='obs_left2', size = [0.2,0.02,0.12])

        # # tutorial.add_box(location=[0.27, -0.375,0.29], name='obs_right2', size = [0.2,0.02,0.18])

        # tutorial.add_box(location=[0.625, 0,0.425], name='grasp_obj', size = [0.05,0.05,0.05])
        tutorial.init_planning_scene_service()
        tutorial.init_ik_service()
        tutorial.view_random_box()
        # tutorial.for_test_show_path_from_start_goal(env_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_env_100_new.npy", 
        #                                             s_g_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_new_s_g_e20_p10000_grasp.npy")
        # tutorial.compare_nnplan_arm(env_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_env_100_new.npy", 
        #                                             s_g_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_new_s_g_e20_p10000.npy")
        # # tutorial.apply_robot_joint_state()
        # tutorial.compare_classical_arm(env_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_env_100_new.npy", 
        #                                 s_g_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/table_case_new_s_g_e20_p10000.npy",
        #                                 len_ref_file="/home/wyhboos/Project/MPMDN/Data/panda_arm/mpmdn_detail.csv")


        # divider environment setup
        # tutorial.add_box(location=[0.4,0,0.2], name='obs_bot', size = [0.4,0.1,0.4])
        # tutorial.add_box(location=[0.6,0,0.6], name='obs_top', size = [0.5,0.02,0.4])

        # tutorial.plan_random_defaut_planner()
        # tutorial.plan_random()
        tutorial.move_group.set_planner_id("RRTConnect")
        
        # tutorial.generate_random_valid_pose_joint_state_for_table_case(100, save_file="/home/wyh/data/table_case_new_pose_100.npy")
        # tutorial.for_test_plan_table_case_vis(pose_file="/home/wyh/data/table_case_pose_100.npy")
        # tutorial.combine_start_goal_for_table_case(pose_file="/home/wyh/data/table_case_pose_100.npy", save_s_g_file="/home/wyh/data/table_case_s_g_60000.npy")

        # parser = argparse.ArgumentParser()
        # parser.add_argument('--thread_index', type=int, default=0)
        # parser.add_argument('_ns', type=str, default=0) #it's tricky, for rosrun to add namespce, without it parser will go wrong
        # args = parser.parse_args()
        # thread_index = args.thread_index
        # print(thread_index)
        # print(args)
        # tutorial.load_scene(file="/home/wyh/data/table_case_env_100_new.npy", index=int(thread_index/2))
        # tutorial.generate_paths_user_plan(s_g_file="/home/wyh/data/table_case_new_s_g_e20_p10000.npy", 
        #                                   path_save_file="/home/wyh/data/table_case_new_BITs.npy", 
        #                                   thread=thread_index, env_index=int(thread_index/2))
        

        # tutorial.generate_paths_user_plan(s_g_file="/home/wyh/data/table_case_s_g_60000.npy", path_save_file="/home/wyh/data/table_case_BIT_star_part", thread=thread_index)
        tutorial.init_ik_service()
        # tutorial.query_inverse_kinematic_service(position=[0.5,0.1,0.5], orientation=[0,0,0,1])

        # tutorial.init_planning_scene_service()
        # tutorial.apply_planing_quest()
        # gen_rand_tbcase_save_new(env_cnt=100, save_file="/home/wyh/data/table_case_env_100_new.npy")
        # tutorial.for_vis_path_with_multiple_start_goal(env_file="/home/wyh/data/table_case_env_100_new.npy", 
        #                                                s_g_file="/home/wyh/data/table_case_new_s_g_e20_p10000.npy")
        # tutorial.gen_pose_start_goal_for_multiple_env(env_file="/home/wyh/data/table_case_env_100_new.npy", 
        #                                               save_s_g_file="/home/wyh/data/table_case_new_s_g_e20_p10000.npy")
        # for i in range(100):
        #     env_index = int(input("Input env index:"))
        #     tutorial.load_scene(file="/home/wyh/data/table_case_env_100_new.npy", index=env_index)
        #     tutorial.show_random_pose(valid_ck=True)
            # tutorial.show_random_pose()
            # print("current pose of end effector is:")
            # end_eff_state = tutorial.move_group.get_current_pose()
            # print(end_eff_state)
            # qua = end_eff_state.pose.orientation
            # print("eular agnle is:", euler_from_quaternion([qua.x,qua.y,qua.z,qua.w]))
            # print("current state of the joint value is:")
            # print(tutorial.move_group.get_current_state())
            # position_input = input("please input the position of end effector:")
            # # orientation_input = input("please input the orientation of end effector:")
            # position = position_input.split(',')
            # position = [float(p) for p in position]
            # # orientation = orientation_input.split(',')
            # # orientation = [float(o) for o in orientation]
            # orientation = [3.141592653589793, -1.7874680211325668e-16, -0.7853981633974488]
            # orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
            # input("============ Press `Enter` to send ...")
            # tutorial.apply_ik_solution(position, orientation)

            # print("current pose of end effector is:")
            # end_eff_state = tutorial.move_group.get_current_pose()
            # print(end_eff_state)
            # qua = end_eff_state.pose.orientation
            # print("eular agnle is:", euler_from_quaternion([qua.x,qua.y,qua.z,qua.w]))
            # print("current state of the joint value is:")
            # print(tutorial.move_group.get_current_state())
            # tutorial.plan_random()
        # tutorial.generate_valid_start_goal_save(cnt=50000, save_file="/home/wyh/start_goal_rrtstar.npy")
        # tutorial.compare_classical()
        # tutorial.generate_paths_user_plan("/home/wyh/start_goal_rrtstar.npy", "/home/wyh/robot_path/path_usr_rrtstar_")
        # for i in range(10):
        #     input("============ Press `Enter` to plan random ...")
        #     tutorial.planner_id = tutorial.move_group.get_planner_id()
        #     tutorial.move_group.set_planner_id("BITstar")
        #     # tutorial.move_group.set_planner_id("RRTstar")

        #     rospy.loginfo("self.planner_id")
        #     print("self.planner_id:", tutorial.planner_id)
        #     tutorial.plan_random()


        # input("============ Press `Enter` to attach a Box to the Panda robot ...")
        # tutorial.attach_box()

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to detach the box from the Panda robot ...")
        # tutorial.detach_box()

        # input(
        #     "============ Press `Enter` to remove the box from the planning scene ..."
        # )
        # tutorial.remove_box()

        # print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


# if __name__ == "__main__":
#     main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
