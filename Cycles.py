# This code is used to gather all the cycles from gait trials.
# The output has lower limb joint angles resampled for every cycles.

# Important packages
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.signal as sg
import os
import math

# Import required files
files_angles = [file for file in os.listdir('dataset') if file.endswith('.csv') and 'angles' in file]
files_traj = [file for file in os.listdir('dataset') if file.endswith('.csv') and 'trajectories' in file]


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


for k in range(len(files_traj)):
    file_traj_id = files_traj[k]

    with open(f'dataset/{file_traj_id}') as file_traj_id:
        # Obtaining name of the file
        name = file_traj_id.name.split('/')[1][0:5]

        # Reading csv trajectories file and making its dataframe
        trajs = pd.read_csv(file_traj_id, header=2)

        # Determining the index of the heel marker
        index = trajs.columns.get_loc("Pilot2:RHEE")

        # Creating RHEE dataframe
        RHEE = trajs.iloc[2:, index:(index + 3)]
        RHEE = pd.DataFrame(np.array(RHEE), columns=["X", "Y", "Z"])

        # To numeric
        cols = RHEE.columns
        RHEE = RHEE[cols].apply(pd.to_numeric, errors='coerce')

        # Find peaks of the signal
        peaks = sg.find_peaks(RHEE["Z"], height=100)

        # Find heel strike times
        t_hs = []
        for i in range(len(peaks[0]) - 1):
            df = RHEE[peaks[0][i]:peaks[0][i + 1]]
            t_hs.append(df["Z"].idxmin(axis=0))

        # Creating cycles
        cycles = []
        for i in range(len(t_hs) - 1):
            cycle = [t_hs[i], t_hs[i + 1], name, 'Right']
            cycles.append(cycle)

        # Same algorithm for left foot cycles
        index = trajs.columns.get_loc("Pilot2:LHEE")
        LHEE = trajs.iloc[2:, index:(index + 3)]
        LHEE = pd.DataFrame(np.array(LHEE), columns=["X", "Y", "Z"])
        cols = LHEE.columns
        LHEE = LHEE[cols].apply(pd.to_numeric, errors='coerce')

        peaks = sg.find_peaks(LHEE["Z"], height=100)

        t_hs = []
        for i in range(len(peaks[0]) - 1):
            df = LHEE[peaks[0][i]:peaks[0][i + 1]]
            t_hs.append(df["Z"].idxmin(axis=0))

        for i in range(len(t_hs) - 1):
            cycle = [t_hs[i], t_hs[i + 1], name, 'Left']
            cycles.append(cycle)

        # Gathering all right and left cycles in a dictionary
        cycles = pd.DataFrame(cycles, columns=['start', 'stop', 'name', 'leg'])

    # Selecting lower limb joint angles in the same gait trial using cycles obtained above
    file_angles_id = files_angles[k]

    # Close the file
    file_traj_id.close()

    with open(f'dataset/{file_angles_id}') as file_angles_id:
        # Reading joint angles csv file and making dataframe
        angles = pd.read_csv(file_angles_id, header=2)

        # Defining joint angles
        joint_angles = {'KneeAngles': [],
                        'AnkleAngles': [],
                        'HipAngles': [],
                        'PelvisAngles': []}

        # Finding joints' indexes for Right leg
        indexR = []
        for i in joint_angles.items():
            index = [i[0], angles.columns.get_loc(f"Pilot2:R{i[0]}")]
            indexR.append(index)

        indexL = []
        for i in joint_angles.items():
            index = [i[0], angles.columns.get_loc(f"Pilot2:L{i[0]}")]
            indexL.append(index)

        # Setting joint angles and resampling each cycle to 101 frames using scale_size function
        for k in range(len(cycles)):
            if cycles.loc[k, 'leg'] == 'Right':
                for index in indexR:
                    temp = angles.loc[cycles.loc[k, 'start']:cycles.loc[k, 'stop'],
                           f"Pilot2:R{index[0]}":f"Unnamed: {index[1] + 2}"].dropna().reset_index(drop=True)

                    joint_angles[f"{index[0]}"].append(scale_size(temp.to_numpy(), 101))

            else:
                for index in indexL:
                    temp = angles.loc[cycles.loc[k, 'start']:cycles.loc[k, 'stop'],
                    f"Pilot2:L{index[0]}":f"Unnamed: {index[1] + 2}"].dropna().reset_index(drop=True)

                    joint_angles[f"{index[0]}"].append(scale_size(temp.to_numpy(), 101))

    # Getting average values of all cycles for every joint and saving the result in a csv file
    joint_angles["KneeAngles_std"] = pd.DataFrame(np.std(joint_angles["KneeAngles"], axis=0))
    joint_angles["AnkleAngles_std"] = pd.DataFrame(np.std(joint_angles["AnkleAngles"], axis=0))
    joint_angles["HipAngles_std"] = pd.DataFrame(np.std(joint_angles["HipAngles"], axis=0))
    joint_angles["PelvisAngles_std"] = pd.DataFrame(np.std(joint_angles["PelvisAngles"], axis=0))

    joint_angles["KneeAngles"] = pd.DataFrame(np.mean(joint_angles["KneeAngles"], axis=0))
    joint_angles["AnkleAngles"] = pd.DataFrame(np.mean(joint_angles["AnkleAngles"], axis=0))
    joint_angles["HipAngles"] = pd.DataFrame(np.mean(joint_angles["HipAngles"], axis=0))
    joint_angles["PelvisAngles"] = pd.DataFrame(np.mean(joint_angles["PelvisAngles"], axis=0))

    joint_angles_DF = pd.DataFrame()
    columns = []
    for i in joint_angles:
        coordinates = ['X', 'Y', 'Z']

        for j in range(len(coordinates)):
            joint_angles_DF[i+'_'+coordinates[j]] = np.array(joint_angles[i])[:, j]

    joint_angles_DF.to_csv('dataset/joint_angles_average.csv', index=False)

    # Saving cycles as a csv file
    cycles_DF = pd.DataFrame(cycles)
    cycles_DF.to_csv('dataset/cycles.csv', index=False)

    # Closing the file
    file_angles_id.close()
