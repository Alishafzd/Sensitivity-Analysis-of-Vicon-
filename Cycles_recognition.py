import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.signal as sg
import os

files = [file for file in os.listdir() if file.endswith('.csv')]

with open(r"gait1_trajectories.csv") as file_id:
    # Reading csv file and making its dataframe
    df = pd.read_csv(file_id, header=2)

    # Determining the index of the heel marker
    index = df.columns.get_loc("Pilot2:RHEE")

    # Creating RHEE dataframe
    RHEE = df.iloc[2:, index:(index + 3)]
    RHEE = pd.DataFrame(np.array(RHEE), columns=["X", "Y", "Z"])

    # To numeric
    cols = RHEE.columns
    RHEE = RHEE[cols].apply(pd.to_numeric, errors='coerce')

    # Find peaks of the signal
    peaks = sg.find_peaks(RHEE["Z"], height=100)

    # Find heel strike times
    t_hs = []
    for i in range(len(peaks[0])-1):
        df_temp = RHEE[peaks[0][i]:peaks[0][i+1]]
        t_hs.append(df_temp["Z"].idxmin(axis=0))

    # Creating cycles
    cycles_R = []
    for i in range(len(t_hs)-1):
        cycle = range(t_hs[i], t_hs[i+1])
        cycles_R.append(cycle)

    # Same algorithm for left foot cycles
    index = df.columns.get_loc("Pilot2:LHEE")
    LHEE = df.iloc[2:, index:(index + 3)]
    LHEE = pd.DataFrame(np.array(LHEE), columns=["X", "Y", "Z"])
    cols = LHEE.columns
    LHEE = LHEE[cols].apply(pd.to_numeric, errors='coerce')

    peaks = sg.find_peaks(LHEE["Z"], height=100)

    t_hs = []
    for i in range(len(peaks[0])-1):
        df_temp = LHEE[peaks[0][i]:peaks[0][i+1]]
        t_hs.append(df_temp["Z"].idxmin(axis=0))

    cycles_L = []
    for i in range(len(t_hs)-1):
        cycle = range(t_hs[i], t_hs[i+1])
        cycles_L.append(cycle)

    