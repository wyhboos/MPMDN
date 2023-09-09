# -*- encoding:utf-8 -*-
import time

import numpy as np
import matplotlib
import plotly.graph_objects as go
import plotly.io as pio

matplotlib.use('Agg')
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import math
import os
import cv2

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


def org_to_img(x, y, pic_shape, ppm):
    x = x + 20
    y = y + 20
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


def plot_rotat_rec_start_goal_2D_three_link(fig, start, goal, pixel_per_meter):
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

    start_link3 = start[2]
    rec_size = [start_link3[1] * pixel_per_meter, start_link3[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(start_link3[2], start_link3[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, start_link3[4]], r=4,
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

    goal_link3 = goal[2]
    rec_size = [goal_link3[1] * pixel_per_meter, goal_link3[0] * pixel_per_meter]
    center_axis1, center_axis2 = org_to_img(goal_link3[2], goal_link3[3], fig.shape, pixel_per_meter)
    fig = plot_rotating_rec_2D(fig=fig, rec_size=rec_size, state=[center_axis1, center_axis2, goal_link3[4]], r=4,
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
    centers_all = [[], []]
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


def plot_rotat_rec_path_2D_three_link(fig, path, pixel_per_meter):
    # plot rectangles
    centers_all = [[], [], []]
    for recs in path:
        for i in range(3):
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


def vis_for_3D_planning_point(cubes, path, save_fig_file):
    # init figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])
    # plot obstacles(cubes)
    plot_multiple_cube_3D(cubes)
    # plot path
    path_x = []
    path_y = []
    path_z = []
    path_len = len(path)
    for i in range(path_len):
        path_x.append(path[i][0])
        path_y.append(path[i][1])
        path_z.append(path[i][2])
    plt.plot(path_x, path_y, path_z, c='green', marker='o')
    # plot initial and goal
    plt.plot(path[0][0], path[0][1], path[0][2], c='b', marker='o')
    plt.plot(path[path_len - 1][0], path[path_len - 1][1], path[path_len - 1][2], c='r', marker='o')

    # save fig
    fig.savefig(save_fig_file + '.png')
    return fig


def plot_multiple_cube_3D(fig, cubes, color='dodgerblue', alpha=0.3):
    """
    :param fig:
    :param cubes: [cube_vertices1, cube_vertices2,..]
    :param color:
    :param alpha:
    :return:
    """
    for i in range(len(cubes)):
        fig = plot_single_cube_3D(fig, cubes[i], color, alpha)
    return fig


def plot_single_cube_3D(fig, vertices, color='dodgerblue', alpha=0.3):
    """
    :param vertices: [[x_min,y_min,z_min], [x_max,y_max,z_max]]
    :param color:
    :param alpha: Opacity with 0 as completely transparent and 1 as opaque
    :return:fig
    """
    # compute 8 vertices
    x_min = vertices[0][0]
    x_max = vertices[1][0]
    y_min = vertices[0][1]
    y_max = vertices[1][1]
    z_min = vertices[0][2]
    z_max = vertices[1][2]
    vertices_all = [[x_min, y_min, z_min], [x_min, y_min, z_max], [x_min, y_max, z_max], [x_min, y_max, z_min],
                    [x_max, y_min, z_min], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_max, y_max, z_min]]
    # faces: front, back, left, right, top, bottom
    faces = [[vertices_all[4], vertices_all[7], vertices_all[6], vertices_all[5]],
             [vertices_all[0], vertices_all[3], vertices_all[2], vertices_all[1]],
             [vertices_all[0], vertices_all[4], vertices_all[5], vertices_all[1]],
             [vertices_all[3], vertices_all[7], vertices_all[6], vertices_all[2]],
             [vertices_all[5], vertices_all[6], vertices_all[2], vertices_all[1]],
             [vertices_all[4], vertices_all[7], vertices_all[3], vertices_all[0]]]

    # plot
    ax = fig.get_axes()[0]
    cube = Poly3DCollection(faces, linewidths=1, edgecolors='k')
    cube.set_facecolor(color)
    cube.set_alpha(alpha)
    ax.add_collection3d(cube)
    return fig


def plot_single_cube_3D_plotly(fig, vertices, color='dodgerblue', alpha=0.3):
    """
    :param vertices: [[x_min,y_min,z_min], [x_max,y_max,z_max]]
    :param color:
    :param alpha: Opacity with 0 as completely transparent and 1 as opaque
    :return:fig
    """
    # compute 8 vertices
    x_min = vertices[0][0]
    x_max = vertices[1][0]
    y_min = vertices[0][1]
    y_max = vertices[1][1]
    z_min = vertices[0][2]
    z_max = vertices[1][2]
    vertices_all = [[x_min, y_min, z_min], [x_min, y_min, z_max], [x_min, y_max, z_max], [x_min, y_max, z_min],
                    [x_max, y_min, z_min], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_max, y_max, z_min]]
    vertices_x = [v[0] for v in vertices_all]
    vertices_y = [v[1] for v in vertices_all]
    vertices_z = [v[2] for v in vertices_all]

    # faces: front, back, left, right, top, bottom
    faces = [[vertices_all[4], vertices_all[7], vertices_all[6], vertices_all[5]],
             [vertices_all[0], vertices_all[3], vertices_all[2], vertices_all[1]],
             [vertices_all[0], vertices_all[4], vertices_all[5], vertices_all[1]],
             [vertices_all[3], vertices_all[7], vertices_all[6], vertices_all[2]],
             [vertices_all[5], vertices_all[6], vertices_all[2], vertices_all[1]],
             [vertices_all[4], vertices_all[7], vertices_all[3], vertices_all[0]]]

    triangles_faces_index_0 = [4, 4, 0, 0, 0, 0, 3, 3, 3, 0, 1, 2]
    triangles_faces_index_1 = [5, 6, 1, 2, 1, 5, 6, 7, 4, 4, 2, 6]
    triangles_faces_index_2 = [6, 7, 2, 3, 5, 4, 2, 6, 7, 3, 5, 5]

    # plot
    color = 'rgba(255, 0, 0,' + str(alpha) + ')'
    cube = go.Mesh3d(
        x=vertices_x,
        y=vertices_y,
        z=vertices_z,
        i=triangles_faces_index_0,
        j=triangles_faces_index_1,
        k=triangles_faces_index_2,
        facecolor=[color] * 12  # 设置面的颜色和透明度
    )
    fig.add_trace(cube)
    return fig


def plot_multiple_cube_3D_plotly(fig, cubes, color='dodgerblue', alpha=0.3):
    """
    :param fig:
    :param cubes: [cube_vertices1, cube_vertices2,..]
    :param color:
    :param alpha:
    :return:
    """
    for i in range(len(cubes)):
        fig = plot_single_cube_3D_plotly(fig, cubes[i], color, alpha)
    return fig

def vis_for_3D_planning_point_plotly(cubes, path, start, goal, save_fig_file):
    # init figure
    layout = go.Layout(
        scene=dict(
            xaxis=dict(showticklabels=False, title='X'),
            yaxis=dict(showticklabels=False, title='Y'),
            zaxis=dict(showticklabels=False, title='Z'),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1)
        )
    )
    fig = go.Figure(layout=layout)

    # plot obstacles(cubes)
    fig = plot_multiple_cube_3D_plotly(fig, cubes, alpha=0.4)
    # plot path
    path_x = []
    path_y = []
    path_z = []
    path_len = len(path)
    for i in range(path_len):
        path_x.append(path[i][0])
        path_y.append(path[i][1])
        path_z.append(path[i][2])
    path_line = go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        marker=dict(
            size=10,
            color='green'
        ),
        line=dict(
            width=10,
            color='green'
        )
    )
    fig.add_trace(path_line)
    # plot initial and goal
    initial = go.Scatter3d(
        x=[start[0]],
        y=[start[1]],
        z=[start[2]],
        marker=dict(
            size=10,
            color='blue'
        ),
    )
    goal = go.Scatter3d(
        x=[goal[0]],
        y=[goal[1]],
        z=[goal[2]],
        marker=dict(
            size=10,
            color='red'
        ),
    )
    fig.add_trace(initial)
    fig.add_trace(goal)

    # save fig
    pio.write_html(fig, save_fig_file + '.html')
    return fig


def vis_for_2D_planning_rigidbody(rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
    # create the fig and plot obstacles
    fig = plot_rotat_rec_env_2D(rec_env, size, pixel_per_meter)
    # plot start and goal state
    fig = plot_rotat_rec_start_goal_2D(fig, start, goal, pixel_per_meter)
    # plot the path
    fig = plot_rotat_rec_path_2D(fig, path, pixel_per_meter)
    print(save_fig_dir)
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


def vis_for_2D_planning_three_link(rec_env, start, goal, path, size, pixel_per_meter, save_fig_dir):
    # create the fig and plot obstacles
    fig = plot_rotat_rec_env_2D(rec_env, size, pixel_per_meter)
    # plot start and goal state
    fig = plot_rotat_rec_start_goal_2D_three_link(fig, start, goal, pixel_per_meter)
    # plot the path
    fig = plot_rotat_rec_path_2D_three_link(fig, path, pixel_per_meter)
    cv2.imwrite(save_fig_dir + '.png', fig)
    return fig

def vis_for_points_in_3D_plotly(points, save_fig_file):
    """
    assume the points is organized as (dimension, cnt)
    """
    layout = go.Layout(
        scene=dict(
            xaxis=dict(showticklabels=False, title='X'),
            yaxis=dict(showticklabels=False, title='Y'),
            zaxis=dict(showticklabels=False, title='Z'),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1)
        )
    )
    fig = go.Figure(layout=layout)
    cnt = points.shape[1]
    for i in range(cnt):
        point = go.Scatter3d(
        x=[points[0,i]],
        y=[points[1,i]],
        z=[points[2,i]],
        marker=dict(
            size=1,
            color='blue'),)
        fig.add_trace(point)
    pio.write_html(fig, save_fig_file)
        

    

if __name__ == '__main__':
    # fig = plt.figure()
    # plot_single_cube_3D_plotly(None, vertices=[[0, 0, 0], [1, 1, 1]])
    vis_for_3D_planning_point_plotly([[[0, 0, 0], [1, 1, 1]], [[2,2,2], [3,3,3]]], [[2,2,2],[1,2,3], [7,5,2]])
    # ax = fig.add_subplot(111, projection='3d')
    # fig, ax = plot_cube_3D(fig, ax, [[0, 0, 0], [5, 5, 5]])
    # fig, ax = plot_cube_3D(fig, ax, [[1, 2, 3], [4, 5, 6]])
    # print(fig)
    # plt.show()
#     rec_obs = [[4, 5, 5, 6, -0.1*math.pi]]
#     plot_rotat_rec_env_start_goal_2D(rec_env=rec_obs, pixel_per_meter=100)
