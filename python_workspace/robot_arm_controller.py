from time import sleep
from math import degrees, pi

import numpy as np

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()

    translate_1 = [0.06, 0, -0.06]
    translate_2 = [-0.06, 0, 0.06]

    test_robot.home()
    sleep(3)

    test_robot.translate_xyz(translate_1[0], translate_1[1], translate_1[2])

    sleep(2)

    test_robot.translate_xyz(translate_2[0], translate_2[1], translate_2[2])

    sleep(2)

    vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    test_robot.set_joint_angles(vert)

    sleep(2)

    


if __name__=="__main__":
    main()