from time import sleep
from math import degrees, pi

import numpy as np
import serial

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()

    TEST_TF_MATRIX = np.array([[1, 0, 0, 0.18],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.05],
                                [0, 0, 0, 1]])
    # TEST_TF_MATRIX = np.array([[0, 0, -1, 0.08],
    #                             [0, 1, 0, 0],
    #                             [1, 0, 0, 0.2],
    #                             [0, 0, 0, 1]])
    test_robot.tf_move(TEST_TF_MATRIX)

    sleep(5)

    translate_1 = np.array([[1, 0, 0, -0.01],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0.02],
                          [0, 0, 0, 1]])
    translate_2 = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0.02],
                          [0, 0, 0, 1]])

    NEW_TF = translate_1 @ TEST_TF_MATRIX
    test_robot.tf_move(NEW_TF)


    sleep(5)

    vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    test_robot.set_joint_angles(vert)


if __name__=="__main__":
    main()