# -*- coding: utf-8 -*-
"""
Created on Sat Jan  1 21:25:01 2021

@author: Ali Shafiezadeh

Vicon Nexus Angles Correction
"""

import numpy as np

# Defining rotation functions
def rotation_x(x, cache=True):
    
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


def rotation_y(x, cache=True):
    
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


def rotation_z(x, cache=True):
    
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

# Defining correction function
def rotation(initial_angles, angles):
    
    m = angles.shape[0]
    
    # Defining initial angles
    psi_0 = initial_angles[0, 2]
    phi_0 = initial_angles[0, 0]
    theta_0 = initial_angles[0, 1]
    
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
    
    # Applying correcton on the data
    R = np.matmul(np.matmul(np.matmul(Ry, Rx), Rz), R0_inv)
    
    return R


# Calculating corrected angles using Euler's orientations    
def correction(R):
    
    m = R.shape[0]
    
    # Calculating the corrected angels
    M = np.array([[-np.arcsin(R[:,1,2])],
                  [np.arctan2(R[:,0,2], R[:,2,2])],
                  [np.arctan2(R[:,1,0], R[:,1,1])]])
    
    corrected_angles = M.T.reshape((m,3))
    
    return corrected_angles


# Defining VNAC model
def vnac(initial_angles, angles):
    
    R = rotation(initial_angles, angles)
    
    corrected_angles = correction(R)
    
    return corrected_angles

