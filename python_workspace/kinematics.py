from math import sin, cos, atan, atan2, pi, sqrt, acos, asin
import numpy as np


# --- FORWARD KINEMATICS ---

def calc_frame_matrix(nx, ny, nz, ox, oy, oz, ax, ay, az, Px, Py, Pz):
    frame_matrix = np.array([[nx, ox, ax, Px],
                            [ny, oy, ay, Py],
                            [nz, oz, az, Pz],
                            [0, 0, 0, 1]])
    
    return frame_matrix


def calc_tfm_matrix(d, theta, a, alpha):
    tfm_matrix = np.array([[cos(theta), -(sin(theta))*(cos(alpha)), (sin(theta))*(sin(alpha)), a*cos(theta)],
                            [sin(theta), (cos(theta))*(cos(alpha)), -(cos(theta))*(sin(alpha)), a*sin(theta)],
                            [0,           sin(alpha),                 cos(alpha),                 d],
                            [0,           0,                          0,                          1]])
    
    return tfm_matrix


def compute_DH_params(joint_angles):
    theta_values = []

    for angle in joint_angles:
        theta_values.append(angle)

    DH_parameters = {
        "joint_1": {
            "d": 121/1000.0,
            "theta": theta_values[0],
            "a": 0,
            "alpha": pi/2
        },
        "joint_2": {
            "d": 0,
            "theta": theta_values[1] + pi/2,
            "a": 172.55/1000.0,
            "alpha": 0
        },
        "joint_3": {
            "d": 0,
            "theta": theta_values[2],
            "a": 172.55/1000.0,
            "alpha": 0
        },
        "joint_4": {
            "d": 0,
            "theta": theta_values[3] + pi/2,
            "a": 0,
            "alpha": pi/2
        },
        "joint_5":  {
            "d": 85/1000.0,
            "theta": theta_values[4],
            "a": 0,
            "alpha": 0
        }
    }

    return DH_parameters


def get_FK_mat(joint_angles):
    DH_parameters = compute_DH_params(joint_angles)

    joint_T_mat_list = []

    for key in DH_parameters:
        joint_T_mat = calc_tfm_matrix(DH_parameters[key]["d"],
                DH_parameters[key]["theta"],
                DH_parameters[key]["a"],
                DH_parameters[key]["alpha"])
        joint_T_mat_list.append(joint_T_mat)

    T_mat = joint_T_mat_list[0] @ joint_T_mat_list[1] @ joint_T_mat_list[2] @ joint_T_mat_list[3] @ joint_T_mat_list[4]

    return T_mat


# --- CALCULATE ROTATION MATRICES ---

def rot_mat_12(theta_1):
    pass


def rot_mat_46(theta_4, theta_5):
    r_11 = -sin(theta_4) * cos(theta_5)
    r_21 = cos(theta_4) * cos(theta_5)
    r_31 = sin(theta_5)
    r_12 = sin(theta_4) * sin(theta_5)
    r_22 = -cos(theta_4) * sin(theta_5)
    r_32 = cos(theta_5)
    r_13 = cos(theta_4)
    r_23 = sin(theta_4)

    rot_mat = np.array([[r_11, r_12, r_13],
                        [r_21, r_22, r_23],
                        [r_31, r_32, 0]])
    
    return rot_mat


def rot_mat_14(theta_1, theta_2, theta_3):
    r_11 = -cos(theta_1) * sin(theta_2) * cos(theta_3) - cos(theta_1) * cos(theta_2) * sin(theta_3)
    r_21 = -sin(theta_1) * sin(theta_2) * cos(theta_3) - sin(theta_1) * cos(theta_2) * sin(theta_3)
    r_31 = cos(theta_2) * cos(theta_3) - sin(theta_2) * sin(theta_3)
    r_12 = cos(theta_1) * sin(theta_2) * sin(theta_3) - cos(theta_1) * cos(theta_2) * cos(theta_3)
    r_22 = sin(theta_1) * sin(theta_2) * sin(theta_3) - sin(theta_1) * cos(theta_2) * cos(theta_3)
    r_32 = -cos(theta_2) * sin(theta_3) - sin(theta_2) * cos(theta_3)
    r_13 = sin(theta_1)
    r_23 = -cos(theta_1)

    rot_mat = np.array([[r_11, r_12, r_13],
                        [r_21, r_22, r_23],
                        [r_31, r_32, 0]])
    
    return rot_mat


def rot_mat_16(tf_matrix):
    pass


# --- INVERSE KINEMATICS ---

def calc_wrist_position(tf_mat):
    rot_mat = tf_mat[:3,:3]
    pos_vec = tf_mat[:3, 3].transpose()

    wx = pos_vec[0] - (0.0601 * rot_mat[:3, 2][0].transpose())
    wy = pos_vec[1] - (0.0601 * rot_mat[:3, 2][1].transpose())
    wz = pos_vec[2] - (0.0601 * rot_mat[:3, 2][2].transpose())

    # print(f'wx: {wx}')
    # print(f'wy: {wy}')
    # print(f'wz: {wz}')

    wrist_pos_vec = np.array([wx, wy, wz])
    
    return wrist_pos_vec


def calc_wrist_orientation(tf_mat):
    rot_mat = tf_mat[:3,:3]

    rot_x = rot_mat[:3, 0].transpose()
    rot_y = rot_mat[:3, 1].transpose()
    rot_z = rot_mat[:3, 2].transpose()

    rot_vecs = [rot_x, rot_y, rot_z]

    return rot_vecs


def calc_theta_1(wrist_pos_vector):
    wy = wrist_pos_vector[1]
    wx = wrist_pos_vector[0]

    theta_1 = atan2(wy, wx)

    return theta_1


def calc_theta_3(wrist_pos_vector):
    wy = wrist_pos_vector[1]
    wx = wrist_pos_vector[0]
    wz = wrist_pos_vector[2]

    a = sqrt((wx**2) + (wy**2))
    b = wz - 0.0991

    numerator = ((a**2) + (b**2) - (0.0991**2) - (0.126**2))
    denominator = 2 * 0.0991 * 0.126

    try:
        theta_3 = acos(numerator/denominator)
    except:
        print("Invalid position???")

    return theta_3


def calc_theta_2(wrist_pos_vector, theta_3):

    wy = wrist_pos_vector[1]
    wx = wrist_pos_vector[0]
    wz = wrist_pos_vector[2]

    a = sqrt((wx**2) + (wy**2))
    b = wz - 0.0991

    numerator = ((0.0991 + 0.126 * cos(theta_3)) * b) + (a * 0.126 * sin(theta_3))
    denominator = (a**2) + (b**2)

    theta_2 = acos(numerator/denominator)

    return theta_2


def calc_joint_angles(tf_matrix):
    wrist_pos = calc_wrist_position(tf_matrix)
    theta_1 = round(calc_theta_1(wrist_pos), 2)
    theta_3 = round(calc_theta_3(wrist_pos), 2)
    theta_2 = round(calc_theta_2(wrist_pos, theta_3), 2)

    position_angles = [theta_1, theta_2, theta_3]

    return position_angles


def calc_orientation_angles(tf_matrix):
    position_angles = calc_joint_angles(tf_matrix)
    rot_mat_16 = tf_matrix[:3, :3]

    rot_matrix_14 = rot_mat_14(position_angles[0], position_angles[1], position_angles[2])
    rot_mat_14_inv = rot_matrix_14.transpose()

    rot_mat_product = rot_mat_14_inv @ (rot_mat_16)

    theta_4 = round(acos(rot_mat_product[0, 2]), 2)
    theta_5 = round(acos(rot_mat_product[2, 1]), 2)

    orientation_angles = [theta_4, theta_5]

    return orientation_angles
