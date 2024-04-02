import time
from math import degrees, pi

import numpy as np
import pyspacemouse

import kinematics as k
import robot_class as robot


def main():
    test_robot = robot.Robot_Arm()
    test_robot.home(2)

    success = pyspacemouse.open(device="SpaceMouse Wireless")

    if success:
        state = pyspacemouse.read()
        initial_time = state.t

        while 1:
            # print(state.x, state.y, state.z)
            state = pyspacemouse.read()
            print(state.buttons[0])
            print(state.buttons[-1])
            if state.buttons[0]==1:
                test_robot.set_gripper(0)
                time.sleep(0.2)
            elif state.buttons[-1]==1:
                test_robot.set_gripper(1)
                time.sleep(0.2)

            # new_time = state.t
            # time_diff = new_time - initial_time
            # x_step = state.y/10
            # z_step = state.z/10
            # print(z_step)
            # if x_step > 0.01 and time_diff > 0.2:
            #     test_robot.move_x(0.01)
            #     initial_time = new_time
            # elif x_step < -0.01 and time_diff > 0.2:
            #     test_robot.move_x(-0.01)
            #     initial_time = new_time
            # elif z_step > 0.01 and time_diff > 0.2:
            #     test_robot.move_z(0.01)
            #     initial_time = new_time
            # elif z_step < -0.01 and time_diff > 0.2:
            #     test_robot.move_z(-0.01)
            #     initial_time = new_time
            # elif 

            # time.sleep(0.2)
                

    # move_delay = 0.5
    # long_delay = 0.8

    # test_robot.home(2)

    # test_robot.move_z(-0.05, move_delay)

    # test_robot.move_z(0.05, move_delay)

    # test_robot.move_z(-0.05, move_delay)

    # test_robot.move_x(0.05, long_delay)

    # test_robot.move_x(-0.05, long_delay)

    # test_robot.home(1)

    # vert = [0.000, 0.000, 0.000, 0.000, 0.000]
    # test_robot.set_joint_angles(vert)

    # sleep(2)
    

if __name__=="__main__":
    main()