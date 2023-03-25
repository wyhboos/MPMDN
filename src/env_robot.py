import fcl
import numpy as np
import math


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
         np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
         np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
         np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
         np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def get_rotation_matrix_from_angle_z(theta):
    matrix = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])
    return matrix


class Env_Robot:
    def __init__(self, robot=None, obs=None):
        self.obstacles = None
        self.robot = None
        self.config_path = None

        # init rectangle robot
        if robot is None:
            self.robot_type = "Rectangle"
            self.robot_size = [1, 2]
            robot_r = np.array([[0.0, -1.0, 0.0],
                                [1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0]])
            robot_t = np.array([0, 0, 0])
            robot_tf = fcl.Transform(robot_t)
            robot_model = fcl.Box(1, 2, 0)
            self.robot = fcl.CollisionObject(robot_model, robot_tf)

        if robot == "2-link":
            self.robot_type = robot
            self.robot_size = [1, 0.1, 1, 0.1]

        # init obstacles
        if obs is None:
            self.obstacles = []
            self.obstacles_vis = []
            for i in range(50):
                obs_i_model = fcl.Box(1, 1, 0)
                obs_i_t = np.array([20, 20, 0]) * np.random.rand(3) + np.array([0, 0, 0])
                # obs_i_t = np.array([10, 10, 0])

                self.obstacles_vis.append([1, 1] + list(obs_i_t)[:2] + [0])
                obs_i_tf = fcl.Transform(obs_i_t)
                obs_i = fcl.CollisionObject(obs_i_model, obs_i_tf)
                self.obstacles.append(obs_i)

    def add_obstacles(self, obstacles):
        self.obstacles = obstacles

    def add_robot(self, robot):
        self.robot = robot

    def is_state_valid_2D(self, state):
        # get robot's state
        x = state.getX()
        y = state.getY()
        t = np.array([x, y, 0])
        # print(x, y)
        angle = state.getYaw()
        quaternion = get_quaternion_from_euler(0, 0, angle)
        matrix = get_rotation_matrix_from_angle_z(angle)
        # set robot's state
        self.robot.setTranslation(t)
        # self.robot.setQuatRotation(quaternion)
        self.robot.setRotation(matrix)

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
        for cfg in path:
            path_with_robot.append(self.robot_size + cfg)
        return path_with_robot
