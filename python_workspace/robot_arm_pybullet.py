from math import sin, cos, pi
import time

import pybullet
import pybullet_data
from ikpy.chain import Chain
import numpy as np

import kinematics as k


physicsClient = pybullet.connect(pybullet.GUI)

ROBOT_ID = pybullet.loadURDF("Robot_Arm_URDF/Robot_Arm_URDF.urdf", useFixedBase=1)
robot_chain = Chain.from_urdf_file("Robot_Arm_URDF/Robot_Arm_URDF.urdf")

NUM_JOINTS = pybullet.getNumJoints(ROBOT_ID)
end_effector_id = 5

TEST_TF_MATRIX = np.array([[1, 0, 0, 0.1],
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
    for i in range(10000):
        test_joint_angles = k.calc_joint_angles(TEST_TF_MATRIX)
        test_ee_angles = k.calc_orientation_angles(TEST_TF_MATRIX)
        
        pybullet.setJointMotorControlArray(bodyIndex=ROBOT_ID,
                                        jointIndices=[0, 1, 2, 3, 4],
                                        controlMode=pybullet.POSITION_CONTROL,
                                        targetPositions=[test_joint_angles[0], test_joint_angles[1], test_joint_angles[2], test_ee_angles[0], test_ee_angles[1]])
        
        JOINT_ANGLES = get_joint_angles()
            
        fk = k.get_FK_mat(JOINT_ANGLES)
        fk_x = fk[:3,3][0]
        fk_y = fk[:3,3][1]
        fk_z = fk[:3,3][2]
        # x_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x + 0.1, fk_y, fk_z], [255,0,0])
        # y_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y + 0.1, fk_z], [0, 255,0])
        z_axis = pybullet.addUserDebugLine([fk_x, fk_y, fk_z], [fk_x, fk_y, fk_z - 0.1], [0,0, 255])
        print(test_joint_angles)
        # pybullet.removeUserDebugItem(x_axis)
        # pybullet.removeUserDebugItem(y_axis)
        pybullet.removeUserDebugItem(z_axis)
        pybullet.stepSimulation()
        time.sleep(1./500.)



    pybullet.disconnect()

if __name__=="__main__":
    main()