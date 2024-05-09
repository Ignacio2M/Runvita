import sys

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backend_bases import MouseButton

from RobotModel.SimpleModel import SimpleModel
from UI.utilsDraw import draw_robot


class PositionMap:

    def __init__(self, robot_model: SimpleModel):
        self._robot_model = robot_model
        self.fig, self.ax = plt.subplots()

        self.step = None

        # set new position end
        self._activate_manual_point = False
        self._temp_manual_pose = np.asarray([0, 0, 0])

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_press_key)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_move)

    # -------------------------------------------
    def _on_click(self, event):
        if self._activate_manual_point and event.button is MouseButton.LEFT:
            self.set_new_goal_pose(np.asarray([event.xdata, event.ydata, self._temp_manual_pose[2]]))
            # Clear values
            self._activate_manual_point = not self._activate_manual_point
            self._temp_manual_pose = np.asarray([0, 0, 0])

    def _on_press_key(self, event):
        sys.stdout.flush()
        if event.key == '.':
            self._activate_manual_point = not self._activate_manual_point
        if self._activate_manual_point:  # Clear values
            self._temp_manual_pose = np.asarray([0, 0, 0])

    def _on_scroll(self, event):
        if self._activate_manual_point:
            increment = 1 if event.button == 'up' else -1
            degree = np.degrees(self._temp_manual_pose[2]) + increment
            self._temp_manual_pose[2] = np.radians(degree)

            # Update Draw
            self._update(nex_pose=False)
            plt.draw()

    def _on_move(self, event):
        if event.inaxes and self._activate_manual_point:
            self._temp_manual_pose = np.asarray([event.xdata, event.ydata, self._temp_manual_pose[2]])
            self._update(nex_pose=False)
            plt.draw()

    def set_new_goal_pose(self, goal_pose):
        self.step = self._robot_model.move_to_goal_pose(self._robot_model.move_robot_model.last_pose, goal_pose)

    # -------------------------------------------

    def _draw_path(self):
        poses_x, poses_y, _ = zip(*self._robot_model.move_robot_model.history_pose)
        self.ax.plot(poses_x, poses_y)

    def draw_points(self):
        self.ax.arrow(self._robot_model.move_robot_model.start_pose[0],
                      self._robot_model.move_robot_model.start_pose[1],
                      np.cos(self._robot_model.move_robot_model.start_pose[2]),
                      np.sin(self._robot_model.move_robot_model.start_pose[2]), color='r', width=0.1)
        if self._robot_model.move_robot_model.goal_pose is not None:
            self.ax.arrow(self._robot_model.move_robot_model.goal_pose[0],
                          self._robot_model.move_robot_model.goal_pose[1],
                          np.cos(self._robot_model.move_robot_model.goal_pose[2]),
                          np.sin(self._robot_model.move_robot_model.goal_pose[2]), color='g', width=0.1)

        if self._activate_manual_point:
            self.ax.arrow(self._temp_manual_pose[0], self._temp_manual_pose[1], np.cos(self._temp_manual_pose[2]),
                          np.sin(self._temp_manual_pose[2]), color='b', width=0.1)

    def _update(self, nex_pose=True, _draw_robot=True, draw_path=True, draw_poses=True):
        plt.cla()
        if nex_pose:
            next(self.step)

        if _draw_robot:
            draw_robot(self.ax, self._robot_model.last_hm)

        if draw_path:
            self._draw_path()

        if draw_poses:
            self.draw_points()

        self.ax.set(xlim=[-20, 20],
                    ylim=[-20, 20])

    def update(self):
        while True:
            if self.step is None:

                while self.step is None:
                    self._update(nex_pose=False)
                    plt.pause(0.01)

            while self.step is not None:
                try:
                    self._update()
                    plt.pause(1)  # TODO Sincronice
                except StopIteration:
                    self.step = None
