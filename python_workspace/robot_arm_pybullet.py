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


def main():
    test_robot = robot.Robot_Arm()

    TEST_TF_MATRIX = np.array([[1, 0, 0, 0.18],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.06],
                                [0, 0, 0, 1]])
    
    # ee_x = [0, 0, 1]
    # ee_y = [0, 1, 0]
    # ee_z = [-1, 0, 0]
    # test_robot.set_ee_axes(ee_x, ee_y, ee_z)

    # target_point = [0.15, 0, 0.12]
    # test_joint_angles = test_robot.move_to_point(target_point)

    # test_joint_angles = test_robot.tf_move(TEST_TF_MATRIX)
    test_joint_angles = test_robot.compute_ik(TEST_TF_MATRIX)



    for i in range(len(test_joint_angles)):
        test_joint_angles[i] = radians(test_joint_angles[i])
    
    print(test_joint_angles)
    
    for i in range(2000):
        
        pybullet.setJointMotorControlArray(bodyIndex=ROBOT_ID,
                                        jointIndices=[0, 1, 2, 3, 4],
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPositions=[test_joint_angles[0], test_joint_angles[1], test_joint_angles[2], test_joint_angles[3], test_joint_angles[4]])
        
        JOINT_ANGLES = get_joint_angles()
            
        fk = k.get_FK_mat(JOINT_ANGLES)
        print(fk)
        # fk_x = fk[:3,3][0]
        # fk_y = fk[:3,3][1]
        # fk_z = fk[:3,3][2]
        # x_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x + 0.1, fk_y, fk_z], [255,0,0])
        # y_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y + 0.1, fk_z], [0, 255,0])
        # z_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y, fk_z - 0.1], [0,0, 255])
        # pybullet.removeUserDebugItem(x_axis)
        # pybullet.removeUserDebugItem(y_axis)
        # pybullet.removeUserDebugItem(z_axis)
        pybullet.stepSimulation()
        time.sleep(1./500.)

    translate_1 = np.array([[1, 0, 0, -0.01],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.02],
                            [0, 0, 0, 1]])
    NEW_TF =  TEST_TF_MATRIX @ translate_1
    test_joint_angles = test_robot.compute_ik(NEW_TF)


    for i in range(2000):
        pybullet.setJointMotorControlArray(bodyIndex=ROBOT_ID,
                                        jointIndices=[0, 1, 2, 3, 4],
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPositions=[test_joint_angles[0], test_joint_angles[1], test_joint_angles[2], test_joint_angles[3], test_joint_angles[4]])
        pybullet.stepSimulation()
        time.sleep(1./500.)


    pybullet.disconnect()

if __name__=="__main__":
    main()