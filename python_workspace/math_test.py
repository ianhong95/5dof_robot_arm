from math import sin, cos, pi

import kinematics as k


theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_4 = 0
theta_5 = 0

DH_PARAMETERS = {
    "joint_1": {
        "d": 68.6,
        "theta": theta_1,
        "a": 0,
        "alpha": 0
    },
    "joint_2": {
        "d": 30.5,
        "theta": theta_2,
        "a": 0,
        "alpha": pi/2
    },
    "joint_3": {
        "d": 0,
        "theta": theta_3 + pi/2,
        "a": 126,
        "alpha": 0
    },
    "joint_4": {
        "d": 0,
        "theta": theta_4,
        "a": 101,
        "alpha": 0
    },
    "joint_5": {
        "d": 0,
        "theta": theta_5,
        "a": 61.01,
        "alpha": pi/2
    }
}

INITIAL_POSITION = [[0, 1, 0, 0],
                    [-1, 0, 0, 0],
                    [0, 0, 1, 387.1],
                    [0, 0, 0, 1]]

ORIGIN = [[1, 0, 0, 0],
          [0, 1, 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

transform = k.get_FK_mat(DH_PARAMETERS)

final = transform.dot(ORIGIN)
    
print("overall tfm matrix:")
print(transform)
print("-----")
print("final frame:")
print(final)