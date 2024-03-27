from math import sin, cos, pi, radians
import time

import pybullet
import pybullet_data
import numpy as np

import kinematics as k
import robot_class as robot


physicsClient = pybullet.connect(pybullet.GUI)

ROBOT_ID = pybullet.loadURDF("Robot_Arm_Assm_URDF_V3/Robot_Arm_Assm_URDF_V3.urdf", useFixedBase=1)

NUM_JOINTS = pybullet.getNumJoints(ROBOT_ID)
end_effector_id = 5

POS_2 = np.array([[1, 0, 0, 0.2],
                  [0, 1, 0, 0.1],
                  [0, 0, -1, 0.1],
                  [0, 0, 0, 1]])


def get_joint_angles():
    joint_angles = []

    for joint_id in range(NUM_JOINTS):
        joint_state = pybullet.getJointState(ROBOT_ID, joint_id)
        joint_position = joint_state[0]     # Index 0 refers to the position value (see docs)
        joint_angles.append(joint_position)

    return joint_angles


def go_to_pos(robot, pos_vec):

    test_joint_angles = []
    test_joint_angles = robot.translate_xyz(pos_vec[0], pos_vec[1], pos_vec[2])

    for i in range(5000):
        pybullet.setJointMotorControlArray(bodyIndex=ROBOT_ID,
                                        jointIndices=[0, 1, 2, 3, 4],
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPositions=[test_joint_angles[0], test_joint_angles[1], test_joint_angles[2], test_joint_angles[3], test_joint_angles[4]])
        pybullet.stepSimulation()
        time.sleep(1./500.)

def apply_frame(robot, frame):
    target_joint_angles = robot.compute_ik(frame)
    target_rads = []

    for i in range(len(target_joint_angles)):
        target_rads.append(radians(target_joint_angles[i]))

    for i in range(2000):
        pybullet.setJointMotorControlArray(bodyIndex=ROBOT_ID,
                                        jointIndices=[0, 1, 2, 3, 4],
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPositions=[target_rads[0], target_rads[1], target_rads[2], target_rads[3], target_rads[4]])

    return target_rads

def main():
    test_robot = robot.Robot_Arm()

    test_robot.home()
    
    # test_robot.current_pos = TEST_TF_MATRIX

    # initial_pos_angles = apply_frame(test_robot, TEST_TF_MATRIX)
    # test_robot.set_joint_angles(initial_pos_angles)
    translate_1 = [0.04, 0, 0.06]
    translate_2 = [-0.04, 0, -0.06]
            
    go_to_pos(test_robot, translate_1)
    go_to_pos(test_robot, translate_2)
        # fk_x = fk[:3,3][0]
        # fk_y = fk[:3,3][1]
        # fk_z = fk[:3,3][2]
        # x_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x + 0.1, fk_y, fk_z], [255,0,0])
        # y_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y + 0.1, fk_z], [0, 255,0])
        # z_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y, fk_z - 0.1], [0,0, 255])
        # pybullet.removeUserDebugItem(x_axis)
        # pybullet.removeUserDebugItem(y_axis)
        # pybullet.removeUserDebugItem(z_axis)


    pybullet.disconnect()

if __name__=="__main__":
    main()