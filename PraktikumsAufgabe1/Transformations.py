import numpy as np


def cos(theta):
    return np.around(np.cos(np.deg2rad(theta)), decimals=14)


def sin(theta):
    return np.around(np.sin(np.deg2rad(theta)), decimals=14)


def rot(theta):
    return np.array([[cos(theta), -sin(theta)],
                     [sin(theta), cos(theta)]])


def rotx(theta):
    return np.array([
        [1, 0, 0],
        [0, cos(theta), -sin(theta)],
        [0, sin(theta), cos(theta)]
    ])


def roty(theta):
    return np.array([
        [cos(theta), 0, sin(theta)],
        [0, 1, 0],
        [-sin(theta), 0, cos(theta)]
    ])


def rotz(theta):
    return np.array([
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta), 0],
        [0, 0, 1]
    ])


def rot2trans(r):
    if r.shape == (2, 2):
        return np.array([
            [r[0, 0], r[0, 1], 0],
            [r[1, 0], r[1, 1], 0],
            [0, 0, 1],
        ])
    elif r.shape == (3, 3):
        return np.array([
            [r[0, 0], r[0, 1], r[0, 2], 0],
            [r[1, 0], r[1, 1], r[1, 2], 0],
            [r[2, 0], r[2, 1], r[2, 2], 0],
            [0, 0, 0, 1],
        ])
    else:
        raise ValueError("I don't know this shape")


def trans(t):
    if len(t) == 2:
        return np.array([
            [1, 0, t[0]],
            [0, 1, t[1]],
            [0, 0, 1],
        ])
    elif len(t) == 3:
        return np.array([
            [1, 0, 0, t[0]],
            [0, 1, 0, t[1]],
            [0, 0, 1, t[2]],
            [0, 0, 0, 1],
        ])
    else:
        raise ValueError("I don't know this tuple size")
