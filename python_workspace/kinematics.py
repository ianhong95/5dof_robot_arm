from math import sin, cos, atan, atan2, pi, sqrt, acos, asin
import numpy as np


# --- PHYSICAL ROBOT PARAMETERS ---
DH_PARAMETERS = {
        "joint_1": {
            "d": 121/1000.0,
            "a": 0,
            "alpha": pi/2
        },
        "joint_2": {
            "d": 0,
            "a": 172.55/1000.0,
            "alpha": 0
        },
        "joint_3": {
            "d": 0,
            "a": 172.55/1000.0,
            "alpha": 0
        },
        "joint_4": {
            "d": 0,
            "a": 0,
            "alpha": pi/2
        },
        "joint_5":  {
            "d": 173/1000.0,
            "a": 0,
            "alpha": 0
        }
    }

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

    computed_DH_params = {
        "joint_1": {
            "d": DH_PARAMETERS["joint_1"]["d"],
            "theta": theta_values[0],
            "a": DH_PARAMETERS["joint_1"]["a"],
            "alpha": DH_PARAMETERS["joint_1"]["alpha"]
        },
        "joint_2": {
            "d": DH_PARAMETERS["joint_2"]["d"],
            "theta": theta_values[1] + pi/2,
            "a": DH_PARAMETERS["joint_2"]["a"],
            "alpha": DH_PARAMETERS["joint_2"]["alpha"]
        },
        "joint_3": {
            "d": DH_PARAMETERS["joint_3"]["d"],
            "theta": theta_values[2],
            "a": DH_PARAMETERS["joint_3"]["a"],
            "alpha": DH_PARAMETERS["joint_3"]["alpha"]
        },
        "joint_4": {
            "d": DH_PARAMETERS["joint_4"]["d"],
            "theta": theta_values[3] + pi/2,
            "a": DH_PARAMETERS["joint_4"]["a"],
            "alpha": DH_PARAMETERS["joint_4"]["alpha"]
        },
        "joint_5":  {
            "d": DH_PARAMETERS["joint_5"]["d"],
            "theta": theta_values[4],
            "a": DH_PARAMETERS["joint_5"]["a"],
            "alpha": DH_PARAMETERS["joint_5"]["alpha"]
        }
    }

    return computed_DH_params


def get_FK_mat(joint_angles):
    computed_DH_parameters = compute_DH_params(joint_angles)

    joint_T_mat_list = []

    for key in computed_DH_parameters:
        joint_T_mat = calc_tfm_matrix(computed_DH_parameters[key]["d"],
                computed_DH_parameters[key]["theta"],
                computed_DH_parameters[key]["a"],
                computed_DH_parameters[key]["alpha"])
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

    # print(pos_vec)

    wx = pos_vec[0] - ((DH_PARAMETERS["joint_5"]["d"]) * rot_mat[:3, 2][0].transpose())
    wy = pos_vec[1] - ((DH_PARAMETERS["joint_5"]["d"]) * rot_mat[:3, 2][1].transpose())
    wz = pos_vec[2] - ((DH_PARAMETERS["joint_5"]["d"]) * rot_mat[:3, 2][2].transpose())

    wrist_pos_vec = np.array([wx, wy, wz])

    print(wrist_pos_vec)

    # print(rot_mat[:3, 2][2].transpose())
    
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
    b = wz - DH_PARAMETERS["joint_1"]["d"]  # wz - d1

    numerator = (a**2) + (b**2) - (DH_PARAMETERS["joint_2"]["a"]**2) - (DH_PARAMETERS["joint_3"]["a"]**2)
    denominator = 2 * DH_PARAMETERS["joint_2"]["a"] * DH_PARAMETERS["joint_3"]["a"]

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
    b = wz - DH_PARAMETERS["joint_1"]["d"]

    numerator = (((DH_PARAMETERS["joint_2"]["a"]) + DH_PARAMETERS["joint_3"]["a"] * cos(theta_3)) * b) + (a * DH_PARAMETERS["joint_3"]["a"] * sin(theta_3))
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


# --- TRANSLATIONS AND ROTATIONS ---

def tf_from_position(position_mtx, current_frame):
    translation_mtx = np.array([[1, 0, 0, position_mtx[0]],
                                [0, 1, 0, position_mtx[1]],
                                [0, 0, 1, position_mtx[2]],
                                [0, 0, 0, 1]])

    new_frame = translation_mtx @ current_frame

    return new_frame


# roll = rotation about x
# pitch = rotation about y
# yaw = rotation about z

def tf_from_roll(roll_angle, current_frame):
    rotation_mtx = np.array([[1, 0, 0, 0],
                             [0, cos(roll_angle), -sin(roll_angle), 0],
                             [0, sin(roll_angle), cos(roll_angle), 0],
                             [0, 0, 0, 1]])

    new_frame = rotation_mtx @ current_frame

    return new_frame


def tf_from_pitch(pitch_angle, current_frame):
    rotation_mtx = np.array([[cos(pitch_angle), 0, sin(pitch_angle), 0],
                             [0, 1, 0, 0],
                             [-sin(pitch_angle), 0, cos(pitch_angle), 0],
                             [0, 0, 0, 1]])
    
    new_frame = rotation_mtx @ current_frame

    return new_frame


def tf_from_yaw(yaw_angle, current_frame):
    rotation_mtx = np.array([[cos(yaw_angle), -sin(yaw_angle), 0, 0],
                             [sin(yaw_angle), cos(yaw_angle), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    
    new_frame = rotation_mtx @ current_frame

    return new_frame