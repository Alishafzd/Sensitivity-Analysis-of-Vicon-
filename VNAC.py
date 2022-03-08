"""
@author: Ali Shafiezadeh

Vicon Nexus Angles Correction
"""

import numpy as np
import pandas as pd
import os
import math

files_angles = [file for file in os.listdir('dataset') if file.endswith('.csv') and 'angles.csv' in file]
# evaluation_data = pd.read_csv('evaluation_data.csv')


def scale_size(In, final_size):
    """
    Defining a function to rescale the size of data series
    The algorithm is based on a linear interpolation between data points
    Inputs are a single column (or row) data series and its required final_size

    :param In: Input data set that we want to change its size
    :param final_size: the final size, this should be noted that the size of all the columns will be changed
    :return: Rescaled array
    """

    N = len(In)
    Out = []
    for j in range(final_size):
        x = j * N / final_size
        X = math.floor(x)
        delta_x = x - X

        if delta_x < 0.0001:
            Out.append(In[X])
        else:
            delta_y = In[X + 1] - In[X]
            Out.append(In[X] + delta_x * delta_y)

    # Convert all values to float and return a single list
    Out = np.array(Out).astype(float)
    return Out


def rotation_x(x):
    """
    Defining rotation matrices

    :param x: x-axis rotation angle
    :return: rotation matrix
    """

    # Calculating number of examples
    m = x.shape[0]

    # Initializing zeroes and ones
    zero_array = np.zeros((m, 1))
    one_array = np.ones((m, 1))

    # Calculating rows
    R1 = np.concatenate((np.cos(x), zero_array, np.sin(x), zero_array), axis=1)
    R2 = np.concatenate((zero_array, one_array, zero_array, zero_array), axis=1)
    R3 = np.concatenate((-np.sin(x), zero_array, np.cos(x), zero_array), axis=1)
    R4 = np.concatenate((zero_array, zero_array, zero_array, one_array), axis=1)

    # Appending rows 
    R = np.concatenate((R1, R2, R3, R4), axis=1).reshape((m, 4, 4))

    return R


def rotation_y(x):
    """
    Defining rotation matrices

    :param x: y-axis rotation angle
    :return: rotation matrix
    """
    # Calculating number of examples
    m = x.shape[0]

    # Initializing zeroes and ones
    zero_array = np.zeros((m, 1))
    one_array = np.ones((m, 1))

    # Calculating rows
    R1 = np.concatenate((one_array, zero_array, zero_array, zero_array), axis=1)
    R2 = np.concatenate((zero_array, np.cos(x), -np.sin(x), zero_array), axis=1)
    R3 = np.concatenate((zero_array, np.sin(x), np.cos(x), zero_array), axis=1)
    R4 = np.concatenate((zero_array, zero_array, zero_array, one_array), axis=1)

    # Appending rows 
    R = np.concatenate((R1, R2, R3, R4), axis=1).reshape((m, 4, 4))

    return R


def rotation_z(x):
    """
    Defining rotation matrices

    :param x: z-axis rotation angle
    :return: rotation matrix
    """
    # Calculating number of examples
    m = x.shape[0]

    # Initializing zeroes and ones
    zero_array = np.zeros((m, 1))
    one_array = np.ones((m, 1))

    # Calculating rows
    R1 = np.concatenate((np.cos(x), -np.sin(x), zero_array, zero_array), axis=1)
    R2 = np.concatenate((np.sin(x), np.cos(x), zero_array, zero_array), axis=1)
    R3 = np.concatenate((zero_array, zero_array, one_array, zero_array), axis=1)
    R4 = np.concatenate((zero_array, zero_array, zero_array, one_array), axis=1)

    # Appending rows 
    R = np.concatenate((R1, R2, R3, R4), axis=1).reshape((m, 4, 4))

    return R


def rotation(initial_angles, angles):
    """
    Defining correction function

    :param initial_angles: average of static angles with shape (3,1)
    :param angles: angles on which we want to perform correction with shape (3,1)
    :return: Rotation matrix
    """
    m = angles.shape[0]

    # Defining initial angles
    psi_0 = initial_angles[2]
    phi_0 = initial_angles[0]
    theta_0 = initial_angles[1]

    # Calculating initial rotation 
    Rz0 = np.array([[np.cos(psi_0), -np.sin(psi_0), 0, 0],
                    [np.sin(psi_0), np.cos(psi_0), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    Ry0 = np.array([[1, 0, 0, 0],
                    [0, np.cos(phi_0), -np.sin(phi_0), 0],
                    [0, np.sin(phi_0), np.cos(phi_0), 0],
                    [0, 0, 0, 1]])
    Rx0 = np.array([[np.cos(theta_0), 0, np.sin(theta_0), 0],
                    [0, 1, 0, 0],
                    [-np.sin(theta_0), 0, np.cos(theta_0), 0],
                    [0, 0, 0, 1]])

    # Calculating inverse matrix of initial angles (R0_inv)
    R0_inv = np.linalg.inv(Ry0.dot(Rx0).dot(Rz0))

    # Defining angles
    psi = angles[:, 2].reshape(m, 1)
    phi = angles[:, 0].reshape(m, 1)
    theta = angles[:, 1].reshape(m, 1)

    # Calculating rotations
    Rx = rotation_x(theta)
    Ry = rotation_y(phi)
    Rz = rotation_z(psi)

    # Applying correction on the data
    R = np.matmul(np.matmul(np.matmul(Ry, Rx), Rz), R0_inv)

    return R


def correction(R):
    """
    Calculating corrected angles using Euler's orientations

    :param R: rotation matrix computed from "rotation" function
    :return: corrected angles for each frame of the motion
    """
    m = R.shape[0]

    # Calculating the corrected angels
    M = np.array([[-np.arcsin(R[:, 1, 2])],
                  [np.arctan2(R[:, 0, 2], R[:, 2, 2])],
                  [np.arctan2(R[:, 1, 0], R[:, 1, 1])]])

    corrected_angles = M.T.reshape((m, 3))

    return corrected_angles


def vnac(initial_angles, angles):
    """
    Defining VNAC model

    :param initial_angles: static angles as initial_angles with shape (3,1)
    :param angles: obtained angles on which we want to perform correction
    :return: corrected angles with shape as angles
    """
    R = rotation(initial_angles, angles)

    corrected_angles = correction(R)

    return corrected_angles


def static_mean():
    """
    Calculating the average of joint angles during the static posture

    :return: average of static values as a dictionary
    """

    with open('dataset/statics.csv') as static_file:
        # Reading static data
        raw_data = pd.read_csv(static_file, header=2)

        # Joints we want to correct
        joints = ['Knee', 'Ankle', 'Hip', 'Pelvis']

        # Creating a dictionary containing all the required static data
        static_data = {}
        for joint in joints:
            for leg in ['R', 'L']:
                index = raw_data.columns.get_loc(f"Pilot2:{leg}{joint}Angles")
                static_data[f'{leg}{joint}Angles_X'] = raw_data.loc[:, f"Pilot2:{leg}{joint}Angles"].mean()
                static_data[f'{leg}{joint}Angles_Y'] = raw_data.loc[:, f"Unnamed: {index + 1}"].mean()
                static_data[f'{leg}{joint}Angles_Z'] = raw_data.loc[:, f"Unnamed: {index + 2}"].mean()

        # Close the file
        static_file.close()

    return static_data


def cycles_average(angles, cycles, name):
    """
    Split cycles from the corrected angles dataset using cycles.csv file and then take the average of them.
    This function should be applied on each trial.

    :param angles: corrected angles for the trial
    :param cycles: cycles.csv file, containing all the available cycles in all the trials
    :param name: name of the trial, in order to check the values in cycles.csv
    :return: average value of corrected angles for the trial, resample to 101 frames
    """

    # Defining joint angles
    joint_angles = {'KneeAngles': [],
                    'AnkleAngles': [],
                    'HipAngles': [],
                    'PelvisAngles': []}

    # The algorithm is based on the following explanation:
    # For each cycle defined in the cycles.csv that matches in the name and leg, we rescale the size and add the results
    # to angles_corrected_sum. At the end, this array will be divided to the number of cycles.
    angles_corrected_sum = np.zeros((101, 12))  # 101 frame and 4 joints with 3 angles for each of them
    counter = 0
    for i in ['Right', 'Left']:
        for k in range(len(cycles)):
            if cycles.loc[k, 'name'] == name and cycles.loc[k, 'leg'] == i:
                columns = [c for c in angles.columns if i[0] in c]
                temp = angles.loc[cycles.loc[k, 'start']:cycles.loc[k, 'stop'], columns]

                angles_corrected_sum += scale_size(temp.to_numpy(), 101)
                counter += 1

    angles_corrected_average = angles_corrected_sum

    # Create columns of all the angles and joints in order to convert the angles_corrected_average to DataFrame
    C = [c[1:] for c in columns]
    angles_corrected_average = pd.DataFrame(angles_corrected_average, columns=C)

    return angles_corrected_average, counter


# Obtain average of joint angles during statics pose
statics = static_mean()

# The final goal is to compute the average of corrections on all the trials and cycles
# Therefore, corrections_average is created to add the results of each trial to it and compute the average based on that
# Also, the following loop will open every gaitn.csv file and compute the average of corrections on that trial
corrections_average = np.zeros((101, 12))  # 101 frame and 4 joints with 3 angles for each of them
counter = 0  # Counting the trials
for k in files_angles:
    # Obtain name of the file
    name = k[0:5]

    # Now let's implement the correction function on each joint
    joints = ['Knee', 'Ankle', 'Hip', 'Pelvis']

    with open(f'dataset/{k}') as file_id:
        # Reading joint angels average during gait
        angles_data = pd.read_csv(file_id, header=2)

        # Gather required data. The algorithm is similar as static_mean function.
        angles = pd.DataFrame()
        for joint in joints:
            for leg in ['R', 'L']:
                index = angles_data.columns.get_loc(f"Pilot2:{leg}{joint}Angles")
                angles[f'{leg}{joint}Angles_X'] = angles_data.loc[:, f"Pilot2:{leg}{joint}Angles"]
                angles[f'{leg}{joint}Angles_Y'] = angles_data.loc[:, f"Unnamed: {index + 1}"]
                angles[f'{leg}{joint}Angles_Z'] = angles_data.loc[:, f"Unnamed: {index + 2}"]

        # The algorithm: Choose one joint and one leg. Gather the related static and angle data. Correct the data.
        angles_corrected = {}
        for joint in joints:
            for leg in ['R', 'L']:
                # First, static angles of a specific joint will be obtained and converted to radians.
                static_temp = np.array([statics[f'{leg}{joint}Angles_X'], statics[f'{leg}{joint}Angles_Y'],
                                                statics[f'{leg}{joint}Angles_Z']])
                static_temp = np.deg2rad(static_temp)

                # Second, the angles (x, y, z) of the joint during gait cycles will be gathered and converted to radians
                angles_temp = angles.loc[:, [f'{leg}{joint}Angles_X', f'{leg}{joint}Angles_Y',
                                             f'{leg}{joint}Angles_Z']].dropna().reset_index(drop=True)
                angles_temp = angles_temp.to_numpy()
                angles_temp = np.deg2rad(angles_temp)

                # Next, vnac function will be applied on the joint, and corrected values will be computed
                temp_corrected = np.rad2deg(vnac(static_temp, angles_temp))

                # At the end, the corrected values will be added to the angles_corrected dictionary
                angles_corrected[f'{leg}{joint}Angles_X'] = temp_corrected[:, 0]
                angles_corrected[f'{leg}{joint}Angles_Y'] = temp_corrected[:, 1]
                angles_corrected[f'{leg}{joint}Angles_Z'] = temp_corrected[:, 2]

    # Save the angles_corrected dictionary as a .csv file
    angles_corrected_DF = pd.DataFrame(angles_corrected)
    angles_corrected_DF.to_csv(f"dataset/{name}_corrected.csv", index=False)

    # Open cycles.csv file in order to compute the average of corrected values over all the cycles in the trial
    cycles_file_id = open('dataset/cycles.csv')
    cycles = pd.read_csv(cycles_file_id)

    # Calculate average corrected values for the trial
    trial_average, cycles_count = cycles_average(angles_corrected_DF, cycles, name)

    # Add the trial_average to corrections_average
    corrections_average += trial_average
    counter += cycles_count

corrections_average /= counter
corrections_average.to_csv('dataset/corrections_average.csv', index=False)
