from time import sleep
from math import degrees

import numpy as np
import serial

import kinematics as k

# --- GLOBAL VARIABLES ---
START_CHAR = b'<'
END_CHAR = b'>'
DELIMITER = b','

SERIAL_OBJ = serial.Serial("/dev/ttyUSB0", 115200)


def main():


    target_tf_matrix = np.array([[1, 0, 0, 0.2],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.1],
                                [0, 0, 0, 1]])
    
    position_angles = k.calc_joint_angles(target_tf_matrix)
    orientation_angles = k.calc_orientation_angles(target_tf_matrix)

    for position_angle_idx in range(len(position_angles)):
        position_angles[position_angle_idx] = round(degrees(position_angles[position_angle_idx]), 2)

    for orientation_angle_idx in range(len(orientation_angles)):
        orientation_angles[orientation_angle_idx] = round(degrees(orientation_angles[orientation_angle_idx]), 2)

    theta_1 = str(position_angles[0]).encode()
    theta_2 = str(position_angles[1]).encode()
    theta_3 = str(position_angles[2]).encode()
    theta_4 = str(orientation_angles[0]).encode()
    theta_5 = str(orientation_angles[1]).encode()

    string_to_send = theta_1 + DELIMITER + theta_2 + DELIMITER + theta_3 + DELIMITER + theta_4 + DELIMITER + theta_5 + END_CHAR
    print(string_to_send)
    SERIAL_OBJ.write(string_to_send)
    SERIAL_OBJ.flush()

    sleep(3)

    target_tf_matrix = np.array([[1, 0, 0, 0.1],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.18],
                                [0, 0, 0, 1]])
    
    position_angles = k.calc_joint_angles(target_tf_matrix)
    orientation_angles = k.calc_orientation_angles(target_tf_matrix)

    for position_angle_idx in range(len(position_angles)):
        position_angles[position_angle_idx] = round(degrees(position_angles[position_angle_idx]), 2)

    for orientation_angle_idx in range(len(orientation_angles)):
        orientation_angles[orientation_angle_idx] = round(degrees(orientation_angles[orientation_angle_idx]), 2)

    theta_1 = str(position_angles[0]).encode()
    theta_2 = str(position_angles[1]).encode()
    theta_3 = str(position_angles[2]).encode()
    theta_4 = str(orientation_angles[0]).encode()
    theta_5 = str(orientation_angles[1]).encode()

    string_to_send = theta_1 + DELIMITER + theta_2 + DELIMITER + theta_3 + DELIMITER + theta_4 + DELIMITER + theta_5 + END_CHAR
    print(string_to_send)
    SERIAL_OBJ.write(string_to_send)
    SERIAL_OBJ.flush()

    target_tf_matrix = np.array([[1, 0, 0, 0.2],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.1],
                                [0, 0, 0, 1]])
    
    position_angles = k.calc_joint_angles(target_tf_matrix)
    orientation_angles = k.calc_orientation_angles(target_tf_matrix)

    for position_angle_idx in range(len(position_angles)):
        position_angles[position_angle_idx] = round(degrees(position_angles[position_angle_idx]), 2)

    for orientation_angle_idx in range(len(orientation_angles)):
        orientation_angles[orientation_angle_idx] = round(degrees(orientation_angles[orientation_angle_idx]), 2)

    theta_1 = str(position_angles[0]).encode()
    theta_2 = str(position_angles[1]).encode()
    theta_3 = str(position_angles[2]).encode()
    theta_4 = str(orientation_angles[0]).encode()
    theta_5 = str(orientation_angles[1]).encode()

    string_to_send = theta_1 + DELIMITER + theta_2 + DELIMITER + theta_3 + DELIMITER + theta_4 + DELIMITER + theta_5 + END_CHAR
    print(string_to_send)
    SERIAL_OBJ.write(string_to_send)
    SERIAL_OBJ.flush()

    sleep(3)

    target_tf_matrix = np.array([[1, 0, 0, 0.1],
                                [0, 1, 0, 0],
                                [0, 0, -1, 0.18],
                                [0, 0, 0, 1]])
    
    position_angles = k.calc_joint_angles(target_tf_matrix)
    orientation_angles = k.calc_orientation_angles(target_tf_matrix)

    for position_angle_idx in range(len(position_angles)):
        position_angles[position_angle_idx] = round(degrees(position_angles[position_angle_idx]), 2)

    for orientation_angle_idx in range(len(orientation_angles)):
        orientation_angles[orientation_angle_idx] = round(degrees(orientation_angles[orientation_angle_idx]), 2)

    theta_1 = str(position_angles[0]).encode()
    theta_2 = str(position_angles[1]).encode()
    theta_3 = str(position_angles[2]).encode()
    theta_4 = str(orientation_angles[0]).encode()
    theta_5 = str(orientation_angles[1]).encode()

    string_to_send = theta_1 + DELIMITER + theta_2 + DELIMITER + theta_3 + DELIMITER + theta_4 + DELIMITER + theta_5 + END_CHAR
    print(string_to_send)
    SERIAL_OBJ.write(string_to_send)
    SERIAL_OBJ.flush()

    sleep(3)

    theta_1 = str(135.0).encode()
    theta_2 = str(135.0).encode()
    theta_3 = str(90.0).encode()
    theta_4 = str(90.0).encode()
    theta_5 = str(90.0).encode()

    string_to_send = theta_1 + DELIMITER + theta_2 + DELIMITER + theta_3 + DELIMITER + theta_4 + DELIMITER + theta_5 + END_CHAR
    print(string_to_send)
    SERIAL_OBJ.write(string_to_send)
    SERIAL_OBJ.flush()


if __name__=="__main__":
    main()