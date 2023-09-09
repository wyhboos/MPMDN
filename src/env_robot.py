import fcl
import numpy as np
import math


def get_rotation_matrix_from_angle_z(theta):
    matrix = np.array([[math.cos(theta), -math.sin(theta), 0],
                       [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])
    return matrix


class Env_Robot:
    def __init__(self, robot_type="Rigidbody_2D", obs=None):
        self.robot_type = robot_type
        self.obstacles = None
        self.robot = None
        self.config_path = None
        self.obstacles_vis = None

        #use small rectangle when type is point!
        if robot_type == "Point_2D":
            self.robot_size = [0.01, 0.01]
            robot_t = np.array([0, 0, 0])
            robot_tf = fcl.Transform(robot_t)
            robot_model = fcl.Box(self.robot_size[0], self.robot_size[1], 0)
            self.robot = fcl.CollisionObject(robot_model, robot_tf)
            matrix = get_rotation_matrix_from_angle_z(0)
            self.robot.setRotation(matrix)
            
        # init rectangle robot
        if robot_type == "Rigidbody_2D":
            self.robot_size = [2, 4]
            robot_t = np.array([0, 0, 0])
            robot_tf = fcl.Transform(robot_t)
            robot_model = fcl.Box(self.robot_size[0], self.robot_size[1], 0)
            self.robot = fcl.CollisionObject(robot_model, robot_tf)

        if robot_type == "Two_Link_2D" or robot_type == "Two_Link_2D_vec":
            # [link1_length, link1_width, link2_length, link2_width]
            self.robot_size = [2.5, 0.1, 2.5, 0.1]
            link1_l = self.robot_size[0]
            link1_w = self.robot_size[1]
            link2_l = self.robot_size[2]
            link2_w = self.robot_size[3]

            link1_t = np.array([0, 0, 0])
            link1_tf = fcl.Transform(link1_t)
            link1_model = fcl.Box(link1_l, link1_w, 0)
            self.link1 = fcl.CollisionObject(link1_model, link1_tf)

            link2_t = np.array([0, 0, 0])
            link2_tf = fcl.Transform(link2_t)
            link2_model = fcl.Box(link2_l, link2_w, 0)
            self.link2 = fcl.CollisionObject(link2_model, link2_tf)

        if robot_type == "Three_Link_2D" or robot_type == "Three_Link_2D_vec":
            # [link1_length, link1_width, link2_length, link2_width]
            self.robot_size = [2.5, 0.1, 2.5, 0.1, 2.5, 0.1]
            link1_l = self.robot_size[0]
            link1_w = self.robot_size[1]
            link2_l = self.robot_size[2]
            link2_w = self.robot_size[3]
            link3_l = self.robot_size[4]
            link3_w = self.robot_size[5]

            link1_t = np.array([0, 0, 0])
            link1_tf = fcl.Transform(link1_t)
            link1_model = fcl.Box(link1_l, link1_w, 0)
            self.link1 = fcl.CollisionObject(link1_model, link1_tf)

            link2_t = np.array([0, 0, 0])
            link2_tf = fcl.Transform(link2_t)
            link2_model = fcl.Box(link2_l, link2_w, 0)
            self.link2 = fcl.CollisionObject(link2_model, link2_tf)

            link3_t = np.array([0, 0, 0])
            link3_tf = fcl.Transform(link3_t)
            link3_model = fcl.Box(link3_l, link3_w, 0)
            self.link3 = fcl.CollisionObject(link3_model, link3_tf)
        
        if robot_type == "Point_3D":
            self.robot_size = [0.01, 0.01, 0.01]
            robot_t = np.array([0, 0, 0])
            robot_tf = fcl.Transform(robot_t)
            robot_model = fcl.Box(self.robot_size[0], self.robot_size[1], self.robot_size[2])
            self.robot = fcl.CollisionObject(robot_model, robot_tf)
            matrix = get_rotation_matrix_from_angle_z(0)
            self.robot.setRotation(matrix)
            
        if robot_type == "panda_arm":
            pass
          
        # init obstacles
        if obs is None:
            self.obstacles = []
            self.obstacles_vis = []
            for i in range(50):
                obs_i_model = fcl.Box(1, 1, 0)
                obs_i_t = np.array([20, 20, 0]) * \
                          np.random.rand(3) + np.array([0, 0, 0])
                # obs_i_t = np.array([10, 10, 0])

                self.obstacles_vis.append([1, 1] + list(obs_i_t)[:2] + [0])
                obs_i_tf = fcl.Transform(obs_i_t)
                obs_i = fcl.CollisionObject(obs_i_model, obs_i_tf)
                self.obstacles.append(obs_i)
        else:
            self.load_rec_obs_2D(obs)

    def get_link_config_2D(self, link_state, link_size):
        """
        :param link_state: [x,y,yaw1,yaw2],note that yaw2 is relative to yaw1
        :param link_size: [link1_length, link1_width, link2_length, link2_width]
        :return:
        """
        if self.robot_type == "Two_Link_2D" or self.robot_type == "Two_Link_2D_vec":
            x = link_state[0]
            y = link_state[1]
            yaw1 = link_state[2]
            yaw2 = link_state[3]

            link1_l = link_size[0]
            link1_w = link_size[1]
            link2_l = link_size[2]
            link2_w = link_size[3]

            link1_state = [x, y, yaw1]
            link2_x = x + 0.5 * link1_l * \
                    math.cos(yaw1) + 0.5 * link2_l * math.cos(yaw1 + yaw2)
            link2_y = y + 0.5 * link1_l * \
                    math.sin(yaw1) + 0.5 * link2_l * math.sin(yaw1 + yaw2)
            link2_yaw = yaw1 + yaw2

            link2_state = [link2_x, link2_y, link2_yaw]
            return [link1_state, link2_state]
        
        if self.robot_type == "Three_Link_2D" or self.robot_type == "Three_Link_2D_vec":
            x = link_state[0]
            y = link_state[1]
            yaw1 = link_state[2]
            yaw2 = link_state[3]
            yaw3 = link_state[4]

            link1_l = link_size[0]
            link1_w = link_size[1]
            link2_l = link_size[2]
            link2_w = link_size[3]
            link3_l = link_size[4]
            link3_w = link_size[5]
            link1_state = [x, y, yaw1]

            link2_x = x + 0.5 * link1_l * \
                    math.cos(yaw1) + 0.5 * link2_l * math.cos(yaw1 + yaw2)
            link2_y = y + 0.5 * link1_l * \
                    math.sin(yaw1) + 0.5 * link2_l * math.sin(yaw1 + yaw2)
            link2_yaw = yaw1 + yaw2
            link2_state = [link2_x, link2_y, link2_yaw]

            link3_x = link2_x + 0.5 * link2_l * \
                    math.cos(yaw1+yaw2) + 0.5 * link3_l * math.cos(yaw1 + yaw2+ yaw3)
            link3_y = link2_y + 0.5 * link2_l * \
                    math.sin(yaw1+yaw2) + 0.5 * link3_l * math.sin(yaw1 + yaw2+ yaw3)
            link3_yaw = yaw1 + yaw2 + yaw3
            link3_state = [link3_x, link3_y, link3_yaw]

            return [link1_state, link2_state, link3_state]

    def is_state_valid(self, state):
        if self.robot_type == "Point_2D":
            x = state[0][0]
            y = state[0][1]
            t = np.array([x, y, 0])
            self.robot.setTranslation(t)

            valid_flag = True
            for obs in self.obstacles:
                dis = fcl.distance(self.robot, obs)
                ret = fcl.collide(self.robot, obs)
                if ret > 0:
                    valid_flag = False
                    break
            return valid_flag

        if self.robot_type == "Rigidbody_2D":
            x = state.getX()
            y = state.getY()
            t = np.array([x, y, 0])
            angle = state.getYaw()
            matrix = get_rotation_matrix_from_angle_z(angle)
            self.robot.setTranslation(t)
            self.robot.setRotation(matrix)

            valid_flag = True
            for obs in self.obstacles:
                dis = fcl.distance(self.robot, obs)
                ret = fcl.collide(self.robot, obs)
                if ret > 0:
                    valid_flag = False
                    break
            return valid_flag

        if self.robot_type == "Two_Link_2D":
            Vec = state[0]
            Angle1 = state[1]
            Angle2 = state[2]
            Vec_X = Vec[0]
            Vec_Y = Vec[1]
            Angle1_Yaw = Angle1.value
            Angle2_Yaw = Angle2.value
            link_states = self.get_link_config_2D(
                [Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw], self.robot_size)

            t_1 = np.array([link_states[0][0], link_states[0][1], 0])
            matrix_1 = get_rotation_matrix_from_angle_z(link_states[0][2])
            self.link1.setTranslation(t_1)
            self.link1.setRotation(matrix_1)

            t_2 = np.array([link_states[1][0], link_states[1][1], 0])
            matrix_2 = get_rotation_matrix_from_angle_z(link_states[1][2])
            self.link2.setTranslation(t_2)
            self.link2.setRotation(matrix_2)

            valid_flag = True
            for obs in self.obstacles:
                # dis = fcl.distance(self.robot, obs)
                ret1 = fcl.collide(self.link1, obs)
                ret2 = fcl.collide(self.link2, obs)
                if ret1 > 0 or ret2 > 0:
                    valid_flag = False
                    break
            return valid_flag
        
        if self.robot_type == "Three_Link_2D":
            Vec = state[0]
            Angle1 = state[1]
            Angle2 = state[2]
            Angle3 = state[3]
            Vec_X = Vec[0]
            Vec_Y = Vec[1]
            Angle1_Yaw = Angle1.value
            Angle2_Yaw = Angle2.value
            Angle3_Yaw = Angle3.value
            link_states = self.get_link_config_2D(
                [Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw, Angle3_Yaw], self.robot_size)

            t_1 = np.array([link_states[0][0], link_states[0][1], 0])
            matrix_1 = get_rotation_matrix_from_angle_z(link_states[0][2])
            self.link1.setTranslation(t_1)
            self.link1.setRotation(matrix_1)

            t_2 = np.array([link_states[1][0], link_states[1][1], 0])
            matrix_2 = get_rotation_matrix_from_angle_z(link_states[1][2])
            self.link2.setTranslation(t_2)
            self.link2.setRotation(matrix_2)

            t_3 = np.array([link_states[2][0], link_states[2][1], 0])
            matrix_3 = get_rotation_matrix_from_angle_z(link_states[2][2])
            self.link3.setTranslation(t_3)
            self.link3.setRotation(matrix_3)

            valid_flag = True
            for obs in self.obstacles:
                # dis = fcl.distance(self.robot, obs)
                ret1 = fcl.collide(self.link1, obs)
                ret2 = fcl.collide(self.link2, obs)
                ret3 = fcl.collide(self.link3, obs)
                if ret1 > 0 or ret2 > 0 or ret3>0:
                    valid_flag = False
                    break
            return valid_flag
        
        if self.robot_type == "Two_Link_2D_vec":
            Vec_X = state[0][0]
            Vec_Y = state[0][1]
            Angle1_Yaw = state[0][2]
            Angle2_Yaw = state[0][3]
            
            if Angle2_Yaw>3.14*0.75 or Angle2_Yaw<-3.14*0.75:
                return False
            
            link_states = self.get_link_config_2D(
                [Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw], self.robot_size)

            t_1 = np.array([link_states[0][0], link_states[0][1], 0])
            matrix_1 = get_rotation_matrix_from_angle_z(link_states[0][2])
            self.link1.setTranslation(t_1)
            self.link1.setRotation(matrix_1)

            t_2 = np.array([link_states[1][0], link_states[1][1], 0])
            matrix_2 = get_rotation_matrix_from_angle_z(link_states[1][2])
            self.link2.setTranslation(t_2)
            self.link2.setRotation(matrix_2)

            valid_flag = True
            for obs in self.obstacles:
                # dis = fcl.distance(self.robot, obs)
                ret1 = fcl.collide(self.link1, obs)
                ret2 = fcl.collide(self.link2, obs)
                if ret1 > 0 or ret2 > 0:
                    valid_flag = False
                    break
            return valid_flag
        
        if self.robot_type == "Three_Link_2D_vec":
            Vec_X = state[0][0]
            Vec_Y = state[0][1]
            Angle1_Yaw = state[0][2]
            Angle2_Yaw = state[0][3]
            Angle3_Yaw = state[0][4]
            if Angle2_Yaw>3.14*0.75 or Angle2_Yaw<-3.14*0.75:
                return False
            if Angle3_Yaw>3.14*0.75 or Angle3_Yaw<-3.14*0.75:
                return False
            link_states = self.get_link_config_2D(
                [Vec_X, Vec_Y, Angle1_Yaw, Angle2_Yaw, Angle3_Yaw], self.robot_size)

            t_1 = np.array([link_states[0][0], link_states[0][1], 0])
            matrix_1 = get_rotation_matrix_from_angle_z(link_states[0][2])
            self.link1.setTranslation(t_1)
            self.link1.setRotation(matrix_1)

            t_2 = np.array([link_states[1][0], link_states[1][1], 0])
            matrix_2 = get_rotation_matrix_from_angle_z(link_states[1][2])
            self.link2.setTranslation(t_2)
            self.link2.setRotation(matrix_2)

            t_3 = np.array([link_states[2][0], link_states[2][1], 0])
            matrix_3 = get_rotation_matrix_from_angle_z(link_states[2][2])
            self.link3.setTranslation(t_3)
            self.link3.setRotation(matrix_3)

            valid_flag = True
            for obs in self.obstacles:
                # dis = fcl.distance(self.robot, obs)
                ret1 = fcl.collide(self.link1, obs)
                ret2 = fcl.collide(self.link2, obs)
                ret3 = fcl.collide(self.link3, obs)
                if ret1 > 0 or ret2 > 0 or ret3>0:
                    valid_flag = False
                    break
            return valid_flag
        
        if self.robot_type == "Point_3D":
            x = state[0][0]
            y = state[0][1]
            z = state[0][2]
            t = np.array([x, y, z])
            self.robot.setTranslation(t)

            valid_flag = True
            for obs in self.obstacles:
                dis = fcl.distance(self.robot, obs)
                ret = fcl.collide(self.robot, obs)
                if ret > 0:
                    valid_flag = False
                    break
            return valid_flag

    def get_config_path_with_robot_info_2D(self, path):
        """
        :param path:[[X,Y,Yaw],[]]
        :return:[[x_length, y_length, X, Y, Yaw]]
        """
        path_with_robot = []
        if self.robot_type == "Point_3D":
            return path
        if self.robot_type == "Point_2D":
            for cfg in path:
                path_with_robot.append(self.robot_size + cfg + [0])
            return path_with_robot
                
        if self.robot_type == "Rigidbody_2D":
            for cfg in path:
                path_with_robot.append(self.robot_size + cfg)
            return path_with_robot
        
        if self.robot_type == "Two_Link_2D" or self.robot_type == "Two_Link_2D_vec":
            for cfg in path:
                state = self.get_link_config_2D(cfg, self.robot_size)
                recs = []
                for i, rec in enumerate(state):
                    recs.append(self.robot_size[2 * i:2 * i + 2] + state[i])
                path_with_robot.append(recs)
            return path_with_robot
        
        if self.robot_type == "Three_Link_2D" or self.robot_type == "Three_Link_2D_vec":
            for cfg in path:
                state = self.get_link_config_2D(cfg, self.robot_size)
                recs = []
                for i, rec in enumerate(state):
                    recs.append(self.robot_size[2 * i:2 * i + 2] + state[i])
                path_with_robot.append(recs)
            return path_with_robot

    def get_list_rec_config_with_robot_from_ompl_state(self, state):
        if self.robot_type == "Point_3D":
            x = state[0]
            y = state[1]
            z = state[2]
            return [x, y, z]

        if self.robot_type == "Point_2D":
            x = state[0]
            y = state[1]
            return self.robot_size + [x, y, 0]
        
        if self.robot_type == "Rigidbody_2D":
            x = state().getX()
            y = state().getY()
            angle = state().getYaw()
            return self.robot_size + [x, y, angle]

        if self.robot_type == "Two_Link_2D" or self.robot_type == "Two_Link_2D_vec":
            x = state[0]
            y = state[1]
            yaw1 = state[2]
            yaw2 = state[3]
            link_states = self.get_link_config_2D([x, y, yaw1, yaw2], self.robot_size)
            recs = []
            for i, rec in enumerate(link_states):
                recs.append(self.robot_size[2 * i:2 * i + 2] + link_states[i])
            return recs
        
        if self.robot_type == "Three_Link_2D" or self.robot_type == "Three_Link_2D_vec":
            x = state[0]
            y = state[1]
            yaw1 = state[2]
            yaw2 = state[3]
            yaw3 = state[4]
            link_states = self.get_link_config_2D([x, y, yaw1, yaw2, yaw3], self.robot_size)
            recs = []
            for i, rec in enumerate(link_states):
                recs.append(self.robot_size[2 * i:2 * i + 2] + link_states[i])
            return recs
        
    def load_rec_obs(self, rec_obs):
        if self.robot_type == "Point_3D":
            self.load_rec_obs_3D(rec_obs)
        else:
            self.load_rec_obs_2D(rec_obs)

    def load_rec_obs_2D(self, rec_obs):
        if self.obstacles is not None:
            self.obstacles.clear()
            self.obstacles_vis.clear()
        obs_cnt, pt_c = rec_obs.shape
        for i in range(obs_cnt):
            l_b_x = rec_obs[i, 0]
            l_b_y = rec_obs[i, 1]
            r_t_x = rec_obs[i, 2]
            r_t_y = rec_obs[i, 3]
            l = r_t_x - l_b_x
            w = r_t_y - l_b_y
            c_x = 0.5*(l_b_x+r_t_x)
            c_y = 0.5 * (l_b_y + r_t_y)
            self.obstacles_vis.append([l, w, c_x, c_y, 0])
            obs_i_model = fcl.Box(l, w, 0)
            obs_i_t = np.array((c_x, c_y, 0))
            obs_i_tf = fcl.Transform(obs_i_t)
            obs_i = fcl.CollisionObject(obs_i_model, obs_i_tf)
            self.obstacles.append(obs_i)
    
    def load_rec_obs_3D(self, rec_obs):
        if self.obstacles is not None:
            self.obstacles.clear()
            self.obstacles_vis.clear()
        obs_cnt = rec_obs.shape[0]
        for i in range(obs_cnt):
            l_b_x = rec_obs[i, 0, 0]
            l_b_y = rec_obs[i, 0, 1]
            l_b_z = rec_obs[i, 0, 2]
            r_t_x = rec_obs[i, 1, 0]
            r_t_y = rec_obs[i, 1, 1]
            r_t_z = rec_obs[i, 1, 2]
            l = r_t_x - l_b_x
            w = r_t_y - l_b_y
            h = r_t_z - l_b_z
            c_x = 0.5 * (l_b_x + r_t_x)
            c_y = 0.5 * (l_b_y + r_t_y)
            c_z = 0.5 * (l_b_z + r_t_z)
            # self.obstacles_vis.append([l, w, h,c_x, c_y, 0])
            obs_i_model = fcl.Box(l, w, h)
            obs_i_t = np.array((c_x, c_y, c_z))
            obs_i_tf = fcl.Transform(obs_i_t)
            obs_i = fcl.CollisionObject(obs_i_model, obs_i_tf)
            self.obstacles.append(obs_i)
            self.obstacles_vis.append([[l_b_x, l_b_y, l_b_z], [r_t_x, r_t_y, r_t_z]])
