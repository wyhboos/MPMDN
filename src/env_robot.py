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

        # init rectangle robot
        if robot_type == "Rigidbody_2D":
            self.robot_size = [1, 2]
            robot_t = np.array([0, 0, 0])
            robot_tf = fcl.Transform(robot_t)
            robot_model = fcl.Box(1, 2, 0)
            self.robot = fcl.CollisionObject(robot_model, robot_tf)

        if robot_type == "Two_Link_2D":
            # [link1_length, link1_width, link2_length, link2_width]
            self.robot_size = [2, 0.1, 2, 0.1]
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

    def get_link_config_two_link_2D(self, link_state, link_size):
        """

        :param link_state: [x,y,yaw1,yaw2],note that yaw2 is relative to yaw1
        :param link_size: [link1_length, link1_width, link2_length, link2_width]
        :return:
        """
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

    def is_state_valid_2D(self, state):

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
            link_states = self.get_link_config_two_link_2D(
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

    def get_config_path_with_robot_info_2D(self, path):
        """
        :param path:[[X,Y,Yaw],[]]
        :return:[[x_length, y_length, X, Y, Yaw]]
        """
        path_with_robot = []
        if self.robot_type == "Rigidbody_2D":
            for cfg in path:
                path_with_robot.append(self.robot_size + cfg)

            return path_with_robot
        if self.robot_type == "Two_Link_2D":
            for cfg in path:
                state = self.get_link_config_two_link_2D(cfg, self.robot_size)
                recs = []
                for i, rec in enumerate(state):
                    recs.append(self.robot_size[2*i:2*i+2] + state[i])
                path_with_robot.append(recs)
            return path_with_robot
