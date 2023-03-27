# -*- encoding:utf-8 -*-
import time

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

import math
import os
import cv2

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


def org_to_img(x, y, pic_shape, ppm):
    aixs1 = pic_shape[0] - 1 - y * ppm
    aixs2 = x * ppm
    return int(aixs1 + 0.5), int(aixs2 + 0.5)


def img_to_org(aixs1, aixs2, pic_shape, ppm):
    x = aixs2 / ppm
    y = pic_shape[0] - 1 - aixs1 / ppm
    return x, y


def coordinate_transform_2D(origin, angle):
    """
    计算新坐标系下的坐标，新坐标系由原坐标系逆时针旋转angle角度得到
    :param origin_x:
    :param origin_y:
    :param angle:
    :return:
    """
    origin_x = origin[0]
    origin_y = origin[1]
    new_x = origin_x * math.cos(angle) + origin_y * math.sin(angle)
    new_y = -origin_x * math.sin(angle) + origin_y * math.cos(angle)
    return np.array([int(new_x + 0.5), int(new_y + 0.5)])


def plot_nearby(pic, aixs1, aixs2, r, color):
    if color == 'red':
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 < aixs1 + i < pic.shape[0] and 0 < aixs2 + j < pic.shape[1]:
                    pic[aixs1 + i, aixs2 + j, 0] = 0
                    pic[aixs1 + i, aixs2 + j, 1] = 0
    if color == 'blue':
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 < aixs1 + i < pic.shape[0] and 0 < aixs2 + j < pic.shape[1]:
                    pic[aixs1 + i, aixs2 + j, 1] = 0
                    pic[aixs1 + i, aixs2 + j, 2] = 0
    if color == 'black':
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 < aixs1 + i < pic.shape[0] and 0 < aixs2 + j < pic.shape[1]:
                    pic[aixs1 + i, aixs2 + j, 0] = 0
                    pic[aixs1 + i, aixs2 + j, 1] = 0
                    pic[aixs1 + i, aixs2 + j, 2] = 0
    if color == 'green':
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 < aixs1 + i < pic.shape[0] and 0 < aixs2 + j < pic.shape[1]:
                    pic[aixs1 + i, aixs2 + j, 0] = 0
                    pic[aixs1 + i, aixs2 + j, 2] = 0
    if color == 'shallow_green':
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 < aixs1 + i < pic.shape[0] and 0 < aixs2 + j < pic.shape[1]:
                    pic[aixs1 + i, aixs2 + j, 0] = 0
                    pic[aixs1 + i, aixs2 + j, 1] = 255
                    pic[aixs1 + i, aixs2 + j, 2] = 127

    return pic


# plot line
def plot_line(pic, plot_aixs, aixs1_range, aixs2_range, r, color):
    if plot_aixs == 1:
        aixs1_start = min(aixs1_range)
        aixs1_end = max(aixs1_range)
        for x in range(aixs1_start, aixs1_end + 1):
            pic = plot_nearby(pic=pic, aixs1=x, aixs2=aixs2_range[0], r=r, color=color)
    else:
        aixs2_start = min(aixs2_range)
        aixs2_end = max(aixs2_range)
        for y in range(aixs2_start, aixs2_end + 2):
            pic = plot_nearby(pic=pic, aixs1=aixs1_range[0], aixs2=y, r=r, color=color)
    return pic


def sample(alpha, sigma, mean):
    alpha = np.array(alpha)
    sigma = np.array(sigma)
    mean = np.array(mean)
    prior = np.random.random()
    k = alpha.shape[0]
    d = len(mean[0])
    for i in range(k):
        if i == 0:
            pro_l = 0
        else:
            pro_l = np.sum(alpha[0: i])
        if i == alpha.shape[0] - 1:
            pro_r = 1
        else:
            pro_r = np.sum(alpha[0: i + 1])
        if pro_l <= prior <= pro_r:
            break
    mean_i = mean[i]

    sigma_i = np.diag([sigma[i] for j in range(d)])
    sample = np.random.multivariate_normal(mean=mean_i, cov=sigma_i)

    return sample


def compute_length_2D(start, end):
    l = ((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2) ** 0.5
    return l


def plot_line_in_fig_2D(fig, start, end, r, color):
    l = compute_length_2D(start, end)
    x_r = end[0] - start[0]
    y_r = end[1] - start[1]
    cnt = int(l + 0.5)
    for i in range(cnt):
        x = start[0] + int(i / l * x_r)
        y = start[1] + int(i / l * y_r)
        plot_nearby(fig, aixs1=x, aixs2=y, r=r, color=color)
    return fig


def plot_rotating_rec_2D(fig, rec_size, state, r, color):
    x_length = rec_size[0]
    y_length = rec_size[1]
    center_x = state[0]
    center_y = state[1]
    center = np.array([center_x, center_y])
    angle = state[2]

    left_bottom = np.array([-0.5 * x_length, -0.5 * y_length])
    left_top = np.array([-0.5 * x_length, 0.5 * y_length])
    right_bottom = np.array([0.5 * x_length, -0.5 * y_length])
    right_top = np.array([0.5 * x_length, 0.5 * y_length])
    left_bottom_t = coordinate_transform_2D(left_bottom, -angle) + center
    left_top_t = coordinate_transform_2D(left_top, -angle) + center
    right_bottom_t = coordinate_transform_2D(right_bottom, -angle) + center
    right_top_t = coordinate_transform_2D(right_top, -angle) + center

    lines = []
    lines.append([left_bottom_t, left_top_t])
    lines.append([left_bottom_t, right_bottom_t])
    lines.append([right_bottom_t, right_top_t])
    lines.append([left_top_t, right_top_t])
    for line in lines:
        fig = plot_line_in_fig_2D(fig, line[0], line[1], r, color)

    return fig


# def plot_rotat_rec_env_start_goal_2D(rec_env, cur_loc, goal_loc, next_loc, pixel_per_meter):
def plot_rotat_rec_env_2D(rec_env, size, pixel_per_meter):
    """
    :param rec_env:[[x_length, y_length, center_x, center_y, angle],[]]
    :param cur_loc:
    :param goal_loc:
    :param next_loc:
    :param pixel_per_meter:
    :return:
    """
    l = size * pixel_per_meter
    fig = 255 * np.ones((int(l * 1), int(l * 1), 3), dtype=np.int8)
    # plot rotating rectangle obstacles
    for rec in rec_env:
        # print(rec)
        # attention coordinate transformation from map to img!
        rec_size = [rec[1] * pixel_per_meter, rec[0] * pixel_per_meter]
        center_axis1, center_axis2 = org_to_img(rec[2], rec[3], fig.shape, pixel_per_meter)
        fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, rec[4]], r=2,
                                   color="black")
    return fig


def plot_rotat_rec_start_goal_2D(fig, start, goal, pixel_per_meter):
    # plot start
    rec_size = [start[1] * pixel_per_meter, start[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(start[2], start[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, start[4]], r=4,
                               color="blue")
    # plot goal
    rec_size = [goal[1] * pixel_per_meter, goal[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(goal[2], goal[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, goal[4]], r=4,
                               color="red")

    return fig

def plot_rotat_rec_start_goal_2D_two_link(fig, start, goal, pixel_per_meter):
    # plot start
    start_link1 = start[0]
    rec_size = [start_link1[1] * pixel_per_meter, start_link1[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(start_link1[2], start_link1[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, start_link1[4]], r=4,
                               color="blue")

    start_link2 = start[1]
    rec_size = [start_link2[1] * pixel_per_meter, start_link2[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(start_link2[2], start_link2[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, start_link2[4]], r=4,
                               color="blue")

    # plot goal
    goal_link1 = goal[0]
    rec_size = [goal_link1[1] * pixel_per_meter, goal_link1[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(goal_link1[2], goal_link1[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, goal_link1[4]], r=4,
                               color="red")

    goal_link2 = goal[1]
    rec_size = [goal_link2[1] * pixel_per_meter, goal_link2[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(goal_link2[2], goal_link2[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, goal_link2[4]], r=4,
                               color="red")


    return fig


def plot_rotat_rec_path_2D(fig, path, pixel_per_meter):
    # plot rectangles
    centers = []
    for rec in path:
        # attention coordinate transformation from map to img!
        rec_size = [rec[1] * pixel_per_meter, rec[0] * pixel_per_meter]
        center_axis1, center_axis2 = org_to_img(rec[2], rec[3], fig.shape, pixel_per_meter)
        centers.append([center_axis1, center_axis2])
        fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, rec[4]], r=2,
                                   color="shallow_green")

    # plot line connecting configurations
    l = len(centers)
    if l >= 2:
        for i in range(l - 1):
            p1 = centers[i]
            p2 = centers[i + 1]
            fig = plot_line_in_fig_2D(fig, start=p1, end=p2, r=1, color='red')

    return fig


def plot_rotat_rec_path_2D_two_link(fig, path, pixel_per_meter):
    # plot rectangles
    centers_all = [[],[]]
    for recs in path:
        for i in range(2):
            rec = recs[i]
            rec_size = [rec[1] * pixel_per_meter, rec[0] * pixel_per_meter]
            center_axis1, center_axis2 = org_to_img(rec[2], rec[3], fig.shape, pixel_per_meter)
            centers_all[i].append([center_axis1, center_axis2])
            fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, rec[4]], r=1,
                                       color="shallow_green")

    # plot line connecting configurations
    for centers in centers_all:
        l = len(centers)
        if l >= 2:
            for i in range(l - 1):
                p1 = centers[i]
                p2 = centers[i + 1]
                fig = plot_nearby(fig, p1[0], p1[1], r=3, color='red')
                fig = plot_line_in_fig_2D(fig, start=p1, end=p2, r=1, color='red')
    return fig


def vis_for_2D_planning_rigidbody(rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
    # create the fig and plot obstacles
    fig = plot_rotat_rec_env_2D(rec_env, size, pixel_per_meter)
    # plot start and goal state
    fig = plot_rotat_rec_start_goal_2D_two_link(fig, start, goal, pixel_per_meter)
    # plot the path
    fig = plot_rotat_rec_path_2D(fig, path, pixel_per_meter)
    cv2.imwrite(save_fig_dir + '.png', fig)
    return fig


def vis_for_2D_planning_two_link(rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
    # create the fig and plot obstacles
    fig = plot_rotat_rec_env_2D(rec_env, size, pixel_per_meter)
    # plot start and goal state
    fig = plot_rotat_rec_start_goal_2D_two_link(fig, start, goal, pixel_per_meter)
    # plot the path
    fig = plot_rotat_rec_path_2D_two_link(fig, path, pixel_per_meter)
    cv2.imwrite(save_fig_dir + '.png', fig)
    return fig

# if __name__ == '__main__':
#     rec_obs = [[4, 5, 5, 6, -0.1*math.pi]]
#     plot_rotat_rec_env_start_goal_2D(rec_env=rec_obs, pixel_per_meter=100)
