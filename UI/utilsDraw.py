import numpy as np


def draw_robot(ax, homogeneous_matrix):
    p0_i = np.array([0, 0, 1]).T
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T
    p4_i = np.array([0.5, 0.25, 1]).T
    p5_i = np.array([0.5, -0.25, 1]).T

    T = homogeneous_matrix
    p0 = np.matmul(T, p0_i)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)
    p4 = np.matmul(T, p4_i)
    p5 = np.matmul(T, p5_i)

    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], 'r-')
    ax.plot([p3[0], p5[0]], [p3[1], p5[1]], 'k-')
    ax.plot([p5[0], p4[0]], [p5[1], p4[1]], 'k-')
    ax.plot([p2[0], p4[0]], [p2[1], p4[1]], 'k-')

    ax.quiver(p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1], angles='xy', scale_units='xy', scale=1, color='b')


def draw_important_point(ax, pose, color='r'):
    ax.arrow(pose[0], pose[1], np.cos(pose[2]), np.sin(pose[2]), color=color, width=0.1)

