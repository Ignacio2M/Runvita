import copy

import numpy as np

from PathFinder.basicPathFinder import PathFinderController
from PathFinder.utils import homogeneousMatrix2d
from RobotModel.MoveModel.speedModel import SpeedModel


class SimpleModel:
    def __init__(self, move_robot_model: SpeedModel, path_finder: PathFinderController):
        self.move_robot_model = move_robot_model
        self.path_finder = path_finder
        
        self._last_hm = homogeneousMatrix2d(self.move_robot_model.start_pose)

        self.time_interval = 1  # 1 segundo

    @property
    def last_hm(self):
        return copy.deepcopy(self._last_hm)
    
    def move_to_goal_pose(self, start_pose, goal_pose):
        new_pose = start_pose
        diff = goal_pose - new_pose
        rho = np.hypot(diff[0], diff[1])
        self.move_robot_model.goal_pose = goal_pose

        while rho > 0.001:
            diff = goal_pose - new_pose

            rho, v, w = self.path_finder.calc_control_command(diff[0], diff[1], new_pose[2], goal_pose[2])
            new_pose, cov = self.move_robot_model.step(v, w, self.time_interval)
            hm = homogeneousMatrix2d(new_pose)
            velocity = np.array([v, w])

            self._last_hm = hm

            yield hm, new_pose, cov, velocity
