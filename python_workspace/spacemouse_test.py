import time
from math import degrees, pi

import numpy as np
import pyspacemouse

import kinematics as k
import robot_class as robot


DEADZONE = 0.02
STEP = 0.05


def main():
    test_robot = robot.Robot_Arm()
    test_robot.home(1)

    success = pyspacemouse.open()

    if success:
        print("Connection success!")
    else:
        print("Connection failed")
        exit

    initial_b1_state = 0
    gripper_state = 0
    initial_b2_state = 0
    initial_time = 0

    while 1:
        state = pyspacemouse.read()
        new_time = state.t
        time_diff = new_time - initial_time

        if time_diff > 0.1:
            state = pyspacemouse.read()
            print(state)

            if state.buttons[0]==1 and initial_b1_state==0 and gripper_state==1:
                test_robot.set_gripper(0)
                initial_b1_state = 0
                gripper_state = 0
            elif state.buttons[0]==1 and initial_b1_state==0 and gripper_state==0:
                test_robot.set_gripper(1)
                initial_b1_state = 0
                gripper_state = 1
            elif state.x > DEADZONE:
                test_robot.move_y(STEP)
            elif state.x < -DEADZONE:
                test_robot.move_y(-STEP)
            elif state.y > DEADZONE:
                test_robot.move_x(STEP)
            elif state.y < -DEADZONE:
                test_robot.move_x(-STEP)
            elif state.z > DEADZONE:
                test_robot.move_z(STEP)
            elif state.z < -DEADZONE:
                test_robot.move_z(-STEP)

            initial_time = new_time
        else:
            initial_time = new_time
    

if __name__=="__main__":
    main()