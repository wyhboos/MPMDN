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
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


class Planning_Setup:
    def __init__(self) -> None:
        self.robot = None
        self.obstcales = None

        # init rectangle robot
        self.robot_type = "Rectangle"
        robot_r = np.array([[0.0, -1.0, 0.0],
                           [1.0,  0.0, 0.0],
                           [0.0,  0.0, 1.0]])
        robot_t = np.array([0, 0, 0])
        robot_tf = fcl.Transform(robot_r, robot_t)
        robot_model = fcl.Box(1, 2, 3)
        self.robot = fcl.CollisionObject(robot_model, robot_tf)

        # init obstacles
        self.obstcales = []
        for i in range(7):
            obs_i_model = fcl.Box(5, 5)
            obs_i_t = np.array(15, 15, 0)*np.random.rand(3) + \
                np.array(2.5, 2.5, 0)
            obs_i_tf = fcl.Transform(obs_i_t)
            obs_i = fcl.CollisionObject(obs_i_model, obs_i_tf)
            self.obstcales.append(obs_i)

    def add_obstacles(self, obstacles):
        self.obstcales = obstacles

    def add_robot(self, robot):
        self.robot = robot

    def is_state_valid_2D(self, state):
        # get robot's state
        x = state[0]
        y = state[1]
        t = np.array(x, y, 0)
        angle = state[2]
        quaternion = get_quaternion_from_euler(0, 0, angle)
        # set robot's state
        self.robot.setTransform(t)
        self.robot.setQuatRotation(quaternion)

        colli_flag = False
        for obs in self.obstcales:
            request = fcl.DistanceRequest()
            result = fcl.DistanceResult()
            dis = fcl.distance(self.robot, obs, request, request)
            if dis < 0.1:
                colli_flag = True
                break
        return colli_flag
