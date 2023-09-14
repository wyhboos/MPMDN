import torch
import numpy as np
import sys

sys.path.append("E:\Study_Doc\Code\mpmdn_train\src")
from visualization import *
from plannet.Motion_model import GMPN_S2D_CLOUD_MDN_Pnet, S2D_MDN_Pnet
from cae.CAE_model import PtNet


def sample(alpha, sigma, mean, sample_cnt):
    alpha = alpha.to('cpu')
    sigma = sigma.to('cpu')
    mean = mean.to('cpu')
    # print(alpha)
    # print(sigma)
    # print(mean)
    alpha = np.array(alpha[0])
    sigma = np.array(sigma[0])
    mean = np.array(mean[0])
    # print(alpha)
    # print(sigma)
    # print(mean)
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
    # i = 10
    mean_i = mean[i]
    #
    # sigma_i = np.array([[sigma[i],0],
    #                     [0, sigma[i]]])
    sigma_i = np.diag([sigma[i] for j in range(d)])
    # print(mean_i.shape)
    # print(sigma_i.shape)
    sample = np.random.multivariate_normal(mean=mean_i, cov=sigma_i, size=(sample_cnt))
    return sample


class planer_model_test:
    def __init__(self, checkpoint_load_file, model_type):
        checkpoint = torch.load(checkpoint_load_file)

        self.model_type = model_type

        # load model
        if self.model_type == "MDN":
            self.pnet = GMPN_S2D_CLOUD_MDN_Pnet(input_size=34, output_size=3, mixture_num=20)
            self.enet = PtNet(dim=3)
            self.enet.load_state_dict(checkpoint['Enet_state_dict'])
            self.pnet.load_state_dict(checkpoint['Pnet_state_dict'])
            self.enet.eval()
            self.pnet.eval()
            self.enet.cuda()
            self.pnet.cuda()

        if self.model_type == "MPN":
            self.pnet = S2D_MDN_Pnet(input_size=34, output_size=3)
            self.enet = PtNet(dim=3)
            self.enet.load_state_dict(checkpoint['Enet_state_dict'])
            self.pnet.load_state_dict(checkpoint['Pnet_state_dict'])
            self.enet.eval()
            self.pnet.train()
            self.enet.cuda()
            self.pnet.cuda()

    def generate_distribution(self, cloud, init, goal, sample_cnt):
        init = np.array(init).reshape(1, -1)
        goal = np.array(goal).reshape(1, -1)
        cloud = cloud.reshape(1, cloud.shape[0], cloud.shape[1])
        if self.model_type == 'MDN':
            with torch.no_grad():
                print(cloud.shape)
                z = self.enet(torch.tensor(cloud, dtype=torch.float32).to('cuda'))
                alpha, sigma, mean = self.pnet(z, torch.tensor(init, dtype=torch.float32).to('cuda'),
                                               torch.tensor(goal, dtype=torch.float32).to('cuda'))
                distri_sample = sample(alpha, sigma, mean, sample_cnt)
                return distri_sample

        if self.model_type == "MPN":
            self.pnet.train()
            distri_sample = []
            with torch.no_grad():
                z = self.enet(torch.tensor(cloud, dtype=torch.float32).to('cuda'))
                for i in range(sample_cnt):
                    next = self.pnet(z, torch.tensor(init, dtype=torch.float32).to('cuda'),
                                     torch.tensor(goal, dtype=torch.float32).to('cuda'))
                    next = next.cpu().detach().numpy()
                    distri_sample.append(next[0])
                distri_sample = np.array(distri_sample)
                return distri_sample

    def plot_distribution(self, obs, init, goal, distribution, save_fig_file):
        fig = init_3D_fig_plotly()
        fig = plot_multiple_cube_3D_plotly(fig, obs)
        fig = plot_multiple_point_3D_plotly(fig, distribution, color="green", size=5)
        fig = plot_multiple_point_3D_plotly(fig, [init], color='blue', size=10)
        fig = plot_multiple_point_3D_plotly(fig, [goal], color='red', size=10)
        save_fig_plotly(fig, save_fig_file)

    def generate_plot_distribution(self, cloud, obs_cubes, init, goal, sample_cnt, save_fig_file):
        distri_sample = self.generate_distribution(cloud, init, goal, sample_cnt)
        distri_sample = list(distri_sample)
        outlier_cnt = 0
        for sample in distri_sample:
            for x in list(sample):
                if x > 30 or x < -30:
                    outlier_cnt += 1
        self.plot_distribution(obs_cubes, init, goal, distri_sample, save_fig_file + "_out_cnt_" + str(outlier_cnt))



def save_cloud_latent_vector():
    cloud_file = "E:/Study_Doc/Code/mpmdn_train/data/train/c3d/c3d_obs_cloud_2000_3_2000_rd.npy"
    cloud = np.load(cloud_file)

    enet = PtNet(dim=3)
    checkpoint_load_file = "E:/Study_Doc/Code/mpmdn_train/data/model/models/MPN_C3D_Point_Joint_1_ckp_126.pt"

    checkpoint = torch.load(checkpoint_load_file)
    enet.load_state_dict(checkpoint['Enet_state_dict'])

    enet.eval()
    enet.cuda()

    latent = []
    for i in range(2000):
        z = enet(torch.tensor(cloud[i:i+1, :, :], dtype=torch.float32).to('cuda'))
        z = z.cpu().detach().numpy()
        z = z.reshape(28)
        latent.append(z)

    latent = np.array(latent)
    print(latent.shape)
    np.save("../Data/MPN_C3D_Point_Joint_1_ckp_126_env_latent_2000.npy", latent)

def test_3D_main():
    cloud_file = "E:/Study_Doc/Code/mpmdn_train/data/train/c3d/c3d_obs_cloud_2000_3_2000_rd.npy"
    cloud = np.load(cloud_file)
    print(cloud.shape)

    cubes_file = "E:/Study_Doc/Code/mpmdn_train/data/train/c3d/c3d_obs_rec_50000.npy"
    cubes = np.load(cubes_file)

    init_goal_file = "../Data/C3D_Pt_sg_ev_not_trivial.npy"
    init_goal = np.load(init_goal_file)
    print(init_goal.shape)

    # checkpoint_load_file = "E:/Study_Doc/Code/mpmdn_train/data/model/models/MDN_C3D_Point_Joint_2_debug_ckp_60.pt"
    checkpoint_load_file = "E:/Study_Doc/Code/mpmdn_train/data/model/models/MPN_C3D_Point_Joint_1_ckp_126.pt"
    model_type = "MPN"
    # init = [-5, -3, 0]
    # goal = [5, 12, -1]
    pt = planer_model_test(checkpoint_load_file=checkpoint_load_file, model_type=model_type)
    for i in range(30):
        env_index = i
        for j in range(5):
            init = init_goal[i][j][0]
            goal = init_goal[i][j][1]
            init_str = str(int(init[0] * 10) / 10) + "_" + str(int(init[1] * 10) / 10) + "_" + str(
                int(init[2] * 10) / 10)
            goal_str = str(int(goal[0] * 10) / 10) + "_" + str(int(goal[0] * 10) / 10) + "_" + str(
                int(goal[0] * 10) / 10)
            save_file = "E:/Study_Doc/Code/MPMDN/Data/vis/MPN_ckp_126/" \
                        + model_type + "_env_" + str(env_index) \
                        + "_init_" + init_str + "_goal_" + goal_str + "_"
            pt.generate_plot_distribution(cloud[env_index, :, :], obs_cubes=cubes[env_index, :, :],
                                          init=init, goal=goal,
                                          sample_cnt=1000,
                                          save_fig_file=save_file)


def test_3D_main_find_multi_modal():
    cloud_file = "E:/Study_Doc/Code/mpmdn_train/data/train/c3d/c3d_obs_cloud_2000_3_2000_rd.npy"
    cloud = np.load(cloud_file)
    print(cloud.shape)

    cubes_file = "E:/Study_Doc/Code/mpmdn_train/data/train/c3d/c3d_obs_rec_50000.npy"
    cubes = np.load(cubes_file)

    init_goal_file = "../Data/C3D_Pt_sg_ev_not_trivial.npy"
    init_goal = np.load(init_goal_file)
    print(init_goal.shape)

    checkpoint_load_file = "E:/Study_Doc/Code/mpmdn_train/data/model/models/MDN_C3D_Point_Joint_2_debug_ckp_200.pt"
    # checkpoint_load_file = "E:/Study_Doc/Code/mpmdn_train/data/model/models/MPN_C3D_Point_Joint_1_ckp_126.pt"
    model_type = "MDN"
    env_index = 1
    init = [4.5, 12, 0]
    goal = [4, -2.5, -1]
    pt = planer_model_test(checkpoint_load_file=checkpoint_load_file, model_type=model_type)
    # init = init_goal[i][j][0]
    # goal = init_goal[i][j][1]
    init_str = str(int(init[0] * 10) / 10) + "_" + str(int(init[1] * 10) / 10) + "_" + str(
        int(init[2] * 10) / 10)
    goal_str = str(int(goal[0] * 10) / 10) + "_" + str(int(goal[1] * 10) / 10) + "_" + str(
        int(goal[2] * 10) / 10)
    save_file = "E:/Study_Doc/Code/MPMDN/Data/vis/find_multimodal/" \
                + model_type + "_env_" + str(env_index) \
                + "_init_" + init_str + "_goal_" + goal_str + "_"
    pt.generate_plot_distribution(cloud[env_index, :, :], obs_cubes=cubes[env_index, :, :],
                                  init=init, goal=goal,
                                  sample_cnt=1000,
                                  save_fig_file=save_file)

if __name__ == '__main__':
    test_3D_main_find_multi_modal()
    # test_3D_main()
    # save_cloud_latent_vector()

