from time import sleep
from math import degrees, pi

import numpy as np

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()
    move_delay = 0.8
    mid_delay = 1.5
    long_delay = 2

    test_robot.home(long_delay)

    test_robot.move_x(0.1, move_delay)
    test_robot.move_x(-0.1, move_delay)
    test_robot.move_y(0.1, move_delay)
    test_robot.move_y(-0.2, move_delay)
    test_robot.move_y(0.1, move_delay)
    test_robot.move_z(0.02, move_delay)
    test_robot.move_z(-0.05, move_delay)

    test_robot.pitch(105, mid_delay)
    # test_robot.yaw(90, move_delay)
    # test_robot.yaw(-90, move_delay)
    test_robot.move_x(0.1, move_delay*2)
    test_robot.move_x(-0.1, move_delay)
    test_robot.pitch(-105, mid_delay)
    test_robot.home(long_delay)


    # test_robot.set_gripper(0)

    # sleep(2)

    # test_robot.set_gripper(1)

    # sleep(2)

    # test_robot.set_gripper(0)


    # test_robot.move_z(-0.02, move_delay)

    # test_robot.move_z(-0.05, move_delay)

    # test_robot.move_x(0.02, long_delay)

    # test_robot.move_x(-0.02, long_delay)

    # test_robot.home(1)

    vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    test_robot.set_joint_angles(vert)

    sleep(2)
    

if __name__=="__main__":
    main()