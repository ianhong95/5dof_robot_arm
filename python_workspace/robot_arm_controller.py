from time import sleep
from math import degrees, pi

import numpy as np

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()

    translate_1 = [0.04, 0, -0.06]
    translate_2 = [-0.04, 0, 0.06]

    test_robot.home()
    sleep(3)

    test_robot.move_z(-0.05)

    sleep(1)

    test_robot.move_z(0.05)

    sleep(1)

    test_robot.move_z(-0.05)

    sleep(1)

    test_robot.move_x(0.05)

    sleep(1)

    test_robot.move_x(-0.05)

    sleep(1)

    vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    test_robot.set_joint_angles(vert)

    sleep(2)
    

if __name__=="__main__":
    main()