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
    # Defining a function to rescale the size of data series
    # The algorithm is based on a linear interpolation between data points
    # Inputs are a single column (or row) data series and its required final_size
    N = len(In)
    Out = []
    for j in range(final_size):
        x = j * N / final_size
        X = math.floor(x)
        delta_x = x - X

        if delta_x < 0.0001:
            Out.append(In[X])
        else:
            delta_y = In[X+1] - In[X]
            Out.append(In[X] + delta_x * delta_y)

    # Convert all values to float and return a single list
    Out = np.array(list(map(lambda x: float(x), Out)))
    return Out


for k in range(len(files_traj)):
    file_traj_id = files_traj[k]

    with open(f'dataset/{file_traj_id}') as file_traj_id:
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
        for i in range(len(peaks[0])-1):
            df = RHEE[peaks[0][i]:peaks[0][i + 1]]
            t_hs.append(df["Z"].idxmin(axis=0))

        # Creating cycles
        cycles_R = []
        for i in range(len(t_hs)-1):
            cycle = range(t_hs[i], t_hs[i+1])
            cycles_R.append(cycle)

        # Same algorithm for left foot cycles
        index = trajs.columns.get_loc("Pilot2:LHEE")
        LHEE = trajs.iloc[2:, index:(index + 3)]
        LHEE = pd.DataFrame(np.array(LHEE), columns=["X", "Y", "Z"])
        cols = LHEE.columns
        LHEE = LHEE[cols].apply(pd.to_numeric, errors='coerce')

        peaks = sg.find_peaks(LHEE["Z"], height=100)

        t_hs = []
        for i in range(len(peaks[0])-1):
            df = LHEE[peaks[0][i]:peaks[0][i + 1]]
            t_hs.append(df["Z"].idxmin(axis=0))

        cycles_L = []
        for i in range(len(t_hs)-1):
            cycle = range(t_hs[i], t_hs[i+1])
            cycles_L.append(cycle)

        # Gathering all right and left cycles in a dictionary
        cycles = {'Right': cycles_R,
                  'Left': cycles_L}

    # Selecting lower limb joint angles in the same gait trial using cycles obtained above
    file_angles_id = files_angles[k]

    with open(f'dataset/{file_angles_id}') as file_angles_id:
        # Reading joint angles csv file and making dataframe
        angles = pd.read_csv(file_angles_id, header=2)

        # Defining joint angles
        joint_angles = {'KneeAngles': [],
                        'AnkleAngles': [],
                        'HipAngles': [],
                        'PelvisAngles': []}

        # Setting joint angles and resampling each cycle to 101 frames using scale_size function
        for k in range(len(cycles_R)):
            joint_angles['KneeAngles'].append(
                scale_size(angles["Pilot2:RKneeAngles"][cycles["Right"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['AnkleAngles'].append(
                scale_size(angles["Pilot2:RAnkleAngles"][cycles["Right"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['HipAngles'].append(
                scale_size(angles["Pilot2:RHipAngles"][cycles["Right"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['PelvisAngles'].append(
                scale_size(angles["Pilot2:RPelvisAngles"][cycles["Right"][k]].dropna().reset_index(drop=True), 101))

        for k in range(len(cycles_L)):
            joint_angles['KneeAngles'].append(
                scale_size(angles["Pilot2:LKneeAngles"][cycles["Left"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['AnkleAngles'].append(
                scale_size(angles["Pilot2:LAnkleAngles"][cycles["Left"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['HipAngles'].append(
                scale_size(angles["Pilot2:LHipAngles"][cycles["Left"][k]].dropna().reset_index(drop=True), 101))
            joint_angles['PelvisAngles'].append(
                scale_size(angles["Pilot2:LPelvisAngles"][cycles["Left"][k]].dropna().reset_index(drop=True), 101))

        joint_angles_DF = pd.DataFrame()
        # Getting average values of all cycles for every joint and saving the result in a csv file
        joint_angles_DF["KneeAngles"] = np.mean(joint_angles["KneeAngles"], axis=0)
        joint_angles_DF["AnkleAngles"] = pd.DataFrame(np.mean(joint_angles["AnkleAngles"], axis=0))
        joint_angles_DF["HipAngles"] = pd.DataFrame(np.mean(joint_angles["HipAngles"], axis=0))
        joint_angles_DF["PelvisAngles"] = pd.DataFrame(np.mean(joint_angles["PelvisAngles"], axis=0))

        joint_angles_DF.to_csv('joint_angles_average.csv', index=False)
