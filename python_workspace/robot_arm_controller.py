from time import sleep
from math import degrees, pi

import numpy as np

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()
    move_delay = 0.5
    long_delay = 1

    test_robot.home(2)

    test_robot.move_z(0.05, move_delay)

    test_robot.set_gripper(0)

    sleep(2)

    test_robot.set_gripper(1)

    sleep(2)

    test_robot.set_gripper(0)


    test_robot.move_z(-0.05, move_delay)

    # test_robot.move_z(-0.05, move_delay)

    test_robot.move_x(0.05, long_delay)

    test_robot.move_x(-0.05, long_delay)

    test_robot.home(1)

    vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    test_robot.set_joint_angles(vert)

    sleep(2)
    

if __name__=="__main__":
    main()