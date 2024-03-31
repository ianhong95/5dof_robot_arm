import time
from math import degrees, radians, pi

import numpy as np
from scipy.spatial.transform import Rotation as R
import serial

import kinematics as k


class Robot_Arm:
    def __init__(self):
        # GENERAL
        self.joint_angles = [0, 0, 0, 0, 0]
        self.num_joints = 5
        self.num_steps = 10.0

        # SERIAL
        self.serial_port = serial.Serial("/dev/ttyUSB0", 115200)    # Find a way to scan and find the right serial port
        self.START_CHAR = b'<'
        self.END_CHAR = b'>'
        self.DELIMITER = b','
        self.delay = 1
        

        # --- INITIALIZE ORIENTATION ---
        self.rot_x = np.array([1, 0, 0])
        self.rot_y = np.array([0, 1, 0])
        self.rot_z = np.array([0, 0, 1])

        # --- INITIALIZE POSITION ---
        self.current_pose = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0.551],
                                     [0, 0, 0, 1]])
        self.current_pos = np.array([self.current_pose[0][3], self.current_pose[1][3], self.current_pose[2][3]])
        self.home_pose = np.array([[1, 0, 0, 0.2],
                                  [0, 1, 0, 0],
                                  [0, 0, -1, 0.18],
                                  [0, 0, 0, 1]])
        self.home_rot_pitch = pi

        print("Class initialized!")

    # --- ROBOT MATH ---

    def compute_fk(self):
        current_tf = k.get_FK_mat(self.joint_angles)
        return current_tf
    

    def compute_ik(self, target_tf_mat):
        position_angles = k.calc_joint_angles(target_tf_mat)
        orientation_angles = k.calc_orientation_angles(target_tf_mat)
        angle_list = []

        for position_angle_idx in range(len(position_angles)):
            position_angles[position_angle_idx] = round(degrees(position_angles[position_angle_idx]), 2)
            self.joint_angles[position_angle_idx] = position_angles[position_angle_idx]
            angle_list.append(position_angles[position_angle_idx])
            
        for orientation_angle_idx in range(len(orientation_angles)):
            orientation_angles[orientation_angle_idx] = round(degrees(orientation_angles[orientation_angle_idx]), 2)
            self.joint_angles[orientation_angle_idx+3] = orientation_angles[orientation_angle_idx]
            angle_list.append(orientation_angles[orientation_angle_idx])

        return angle_list


    def compute_tf_mat(self, roll, pitch, yaw, pos_vec):    # roll should 0 because there's no roll motor.
        rot_vec = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        rot_mat = R.as_matrix(rot_vec)
        print(f"rot_mat: {rot_mat}")

        # Build transformation matrix from rotation matrix and position vector
        tf_mat = np.array([[rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], pos_vec[0]],
                           [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], pos_vec[1]],
                           [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], pos_vec[2]],
                           [0, 0, 0, 1]])
        
        return tf_mat


    # --- COMMUNICATION FUNCTIONS ---

    def angles_to_bytes(self):
        for i in range(self.num_joints):
            self.joint_angles[i] = str(self.joint_angles[i]).encode()
        
    def encode_data(self, joint_angle_list):
        encoded_list = []

        for angle in joint_angle_list:
            encoded_list.append(str(angle).encode())

        return encoded_list
    

    def serial_write(self, raw_angle_list):
        encoded_angle_list = self.encode_data(raw_angle_list)

        string_to_send = encoded_angle_list[0] + self.DELIMITER + encoded_angle_list[1] + self.DELIMITER + encoded_angle_list[2] + self.DELIMITER + encoded_angle_list[3] + self.DELIMITER + encoded_angle_list[4] + self.END_CHAR
        print(string_to_send)
        self.serial_port.write(string_to_send)
        self.serial_port.flush()


    # --- GO TO PRE-DEFINED POSITIONS ---
    def home(self, delay=0):
        target_joint_angles = self.compute_ik(self.home_pose)
        
        self.serial_write(target_joint_angles)

        self.current_pose = self.home_pose
        self.update_current_pose(self.current_pose)
        print(f'current position: {self.current_pos}')

        time.sleep(delay)

        return target_joint_angles

    # --- ABSOLUTE MOTION FUNCTIONS ---
        
    def set_joint_angles(self,input_angles):
        joint_angle_list = []
        for i in range(len(input_angles)):
            self.joint_angles[i] = input_angles[i]
            joint_angle_list.append(self.joint_angles[i])

        self.serial_write(joint_angle_list)


    def move_to_point(self, target_abs_coords):
        tf_mat = np.array([[self.rot_x[0], self.rot_y[0], self.rot_z[0], target_abs_coords[0]],
                           [self.rot_x[1], self.rot_y[1], self.rot_z[1], target_abs_coords[1]],
                           [self.rot_x[2], self.rot_y[2], self.rot_z[2], target_abs_coords[2]],
                           [0, 0, 0, 1]])
        
        target_joint_angles = self.tf_move(tf_mat)

        return target_joint_angles


    def set_orientation_rpy(self, roll, pitch, yaw):
        pass


    def set_ee_axes(self, x, y, z):
        self.rot_x = x
        self.rot_y = y
        self.rot_z = z


    def tf_move(self, tf_mat):
        target_joint_angles = self.compute_ik(tf_mat)

        self.serial_write(self.joint_angles)

        # TO DO: update orientation vectors


        return target_joint_angles

    # --- RELATIVE MOTION FUNCTIONS ---

    def translate_xyz(self, x, y, z, delay=0):
        translation_vector = np.array([x, y, z])
        target_rads = []

        new_frame_mat = k.tf_from_position(translation_vector, self.current_pose)
        target_joint_angles = self.compute_ik(new_frame_mat)

        self.serial_write(target_joint_angles)

        for i in range(len(target_joint_angles)):
            target_rads.append(radians(target_joint_angles[i]))

        self.update_current_pose(new_frame_mat)
        print(f'current position: {self.current_pos}')

        time.sleep(delay)

        return target_rads
    

    # --- Simple relative move commands in mm---

    def move_x(self, x_increment, delay=0):
        translation_vector = np.array([x_increment, 0, 0])
        new_frame_mat = k.tf_from_position(translation_vector, self.current_pose)
        target_joint_angles = self.compute_ik(new_frame_mat)

        self.update_current_pose(new_frame_mat)

        self.serial_write(target_joint_angles)
        time.sleep(delay)


    def move_y(self, y_increment, delay=0):
        translation_vector = np.array([0, y_increment, 0])
        new_frame_mat = k.tf_from_position(translation_vector, self.current_pose)
        target_joint_angles = self.compute_ik(new_frame_mat)

        self.update_current_pose(new_frame_mat)

        self.serial_write(target_joint_angles)
        time.sleep(delay)

    
    def move_z(self, z_increment, delay=0):
        translation_vector = np.array([0, 0, z_increment])
        new_frame_mat = k.tf_from_position(translation_vector, self.current_pose)
        target_joint_angles = self.compute_ik(new_frame_mat)

        self.update_current_pose(new_frame_mat)

        self.serial_write(target_joint_angles)
        time.sleep(delay)
    

    # --- Sync all motors to arrive at target at the same time

    def calc_joint_steps(self, current_joint_angles, target_joint_angles):
        angle_step_list = []
        for angle_idx in range(len(current_joint_angles)):
            angle_step = (target_joint_angles[angle_idx] - current_joint_angles[angle_idx]) / self.num_steps
            angle_step_list.append(angle_step)

        return angle_step_list
    

    # --- Misc Functions

    def update_current_pose(self, pose):
        self.current_pose = pose

        print(pose)

        self.current_pos[0] = pose[0][3]
        self.current_pos[1] = pose[1][3]
        self.current_pos[2] = pose[2][3]