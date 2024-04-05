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
        print("Connection success!")
    else:
        print("Connection failed")
        exit

    state = pyspacemouse.read()
    initial_time = state.t

    initial_b1_state = 0
    gripper_state = 0
    initial_b2_state = 0

    while 1:
        new_time = state.t
        time_diff = new_time - initial_time

        if time_diff > 0.1:
            state = pyspacemouse.read()

            if state.buttons[0]==1 and initial_b1_state==0 and gripper_state==1:
                test_robot.set_gripper(0)
                initial_b1_state = 1
                gripper_state = 0
            elif state.buttons[0]==1 and initial_b1_state==0 and gripper_state==0:
                test_robot.set_gripper(1)
                initial_b1_state = 1
                gripper_state = 1
            elif state.x > 0:
                test_robot.move_x(0.01)
            elif state.x < 0:
                test_robot.move_x(-0.01)
            elif state.y > 0:
                test_robot.move_y(0.01)
            elif state.y < 0:
                test_robot.move_y(-0.01)
            elif state.z > 0:
                test_robot.move_z(0.01)
            elif state.z < 0:
                test_robot.move_z(-0.01)

            initial_time = state.t
    

if __name__=="__main__":
    main()