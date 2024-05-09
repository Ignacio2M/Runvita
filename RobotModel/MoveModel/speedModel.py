import copy

import numpy as np


class SpeedModel:
    """
                         distance_wheel
                ├────────────────────────────────┤
                      ┌────────()────────┐
             ┬ ┌─────┐│                  │┌─────┐
wheel_radius │ │     ││                  ││     │
             ┴ │ W_l ││        x         ││ w_r │
               │     ││                  ││     │
               └─────┘│                  │└─────┘
                      └──────────────────┘

    cov_move -> [[e_v 0] [0 e_w]]

    command_interface -> is a interface to communicate with robot


    max_speed_wheel -> float radians/second
    """

    def __init__(self, distance_wheel, wheel_radius, start_pose=None, cov=None, real_map=None):

        self._distance_wheel = distance_wheel
        self._wheel_radius = wheel_radius

        self._start_pose = start_pose if start_pose is not None else np.asarray([0, 0, 0])
        self._cov_move = cov

        self._cov_pose = np.diag([0, 0, 0])
        self._history_poses = [self._start_pose]
        self._history_cov_pose = [self._cov_pose]

        self._real_map = real_map
        self.goal_pose = None

    @property
    def history_pose(self):
        return copy.deepcopy(self._history_poses)

    @property
    def history_cov_pose(self):
        return copy.deepcopy(self._history_cov_pose)

    @property
    def last_pose(self):
        return self._history_poses[-1]

    @property
    def start_pose(self):
        return self._start_pose.copy()

    def _covariance_evol(self, v, w, dt):
        actual_pose = self._history_poses[-1]

        sx, cx = np.sin(actual_pose[2]), np.cos(actual_pose[2])  # sin and cos for the previous robot heading
        si, ci = np.sin(v * dt), np.cos(w * dt)  # sin and cos for the heading increment
        R = v / w  # v/w Curvature radius

        if w == 0:  # linear motion w=0 --> R = infinite

            JacF_x = np.array([
                [1, 0, -v * dt * sx],
                [0, 1, v * dt * cx],
                [0, 0, 1]
            ])
            JacF_u = np.array([
                [dt * cx, 0],
                [dt * sx, 0],
                [0, 0]
            ])
        else:  # Non-linear motion w=!0

            JacF_x = np.array([
                [1, 0, R * (-sx * si - cx * (1 - ci))],
                [0, 1, R * (cx * si - sx * (1 - ci))],
                [0, 0, 1]
            ])

            JacF_u = (
                    np.array([
                        [cx * si - sx * (1 - ci), R * (cx * ci - sx * si)],
                        [sx * si - cx * (1 - ci), R * (sx * ci - cx * si)],
                        [0, 1]
                    ]) @
                    np.array([
                        [1 / w, -v / (w ** 2)],
                        [0, dt]
                    ])
            )
        Pt = (JacF_x @ self._cov_pose @ JacF_x.T) + (JacF_u @ self._cov_move @ JacF_u.T)

        return Pt

    def step(self, v, w, dt):
        actual_pose = self._history_poses[-1]
        dvw = np.array([v, w])

        if self._cov_move is not None:
            self._cov_pose = self._covariance_evol(v, w, dt)
            self._history_cov_pose.append(self._cov_pose)
            # dvw = np.random.multivariate_normal(dvw, self._cov_move)

        if dvw[1] == 0:  # linear motion w=0
            increment = dvw * dt * np.asarray([np.cos(actual_pose[2]), np.sin(actual_pose[2]), 0])
        else:
            R = dvw[0] / dvw[1]  # v/w=r is the curvature radius
            d_theta = dvw[1] * dt
            increment = np.vstack([
                -R * np.sin(actual_pose[2]) + R * np.sin(actual_pose[2] + d_theta),
                R * np.cos(actual_pose[2]) - R * np.cos(actual_pose[2] + d_theta),
                d_theta
            ])
        new_pose = actual_pose + increment.T
        new_pose = new_pose.reshape(3, )

        if self._real_map is not None and self._real_map.collision(new_pose[:2], self._distance_wheel / 2):
            # TODO: Add real time feedback
            raise NotImplementedError("Collision detect")

        self._history_poses.append(new_pose)


        return new_pose, self._cov_pose
