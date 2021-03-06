#! usr/bin/env python3

# takes in the calibrated translation and rotation matrix with the recorded transform from the arm and gives the
# location of the end effector in the world frame.

import numpy as np
import csv
import sys
import os
import pandas as pd

directory = os.path.dirname(os.path.realpath(__file__))

# sets up matrices to read the stored matrices into
tran_mat = np.zeros((4, 4))
translation_mat = np.zeros((4, 4))
rotation_mat = np.zeros((4, 4))

########################################################################################################################
# Read in the translation rotation and transformation matrices
########################################################################################################################

with open(directory + '/TranslationMatrix.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            translation_mat[j][i] = float(col)


with open(directory + '/RotationMatrix.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            rotation_mat[j][i] = float(col)


with open(directory + '/test_data/Matrices/TransformMatrix_9.0.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            tran_mat[j][i] = float(col)


########################################################################################################################
# Calculate the end effector location in the world frame
########################################################################################################################

loc = rotation_mat @ translation_mat @ tran_mat @ np.transpose([0, 0, 0, 1])
print('The location of the end effector')
print(loc)
print('\n')

# load the ArUco Markers information in world frame
ArUco_data = pd.read_csv('data_file_9.csv')
# read the palm's location
palm_data = ArUco_data[ArUco_data.location == 'palm']
palm_data = np.array(palm_data)
Palm_in_ArUco = np.append(palm_data[0, [1, 2, 3]], [1])
print('Palm: ArUco Marker Location')
print(Palm_in_ArUco)
print('\n')

# load transform matrices
EE_to_P = pd.read_csv('EE_to_Palm.csv', header=None)
EE_to_P = np.array(EE_to_P)

P_to_world = pd.read_csv('P_to_world.csv', header=None)
P_to_world = np.array(P_to_world)

# create the translation matrix of end effector from end effector frame to world frame
move = np.eye(4)
move[:, 3] = loc.T

# calculate the location of the palm in the world frame by using matrices
Palm_in_world = move @ P_to_world @ EE_to_P @ [0, 0, 0, 1]
print('Palm: World frame Location')
print(Palm_in_world)
print('\n')

# calculate the difference between the Palm's location in ArUco Marker and the real world
diff_in_world = Palm_in_ArUco - Palm_in_world
diff_in_world[-1] = 1
print(diff_in_world)
# save the difference as a csv file
np.savetxt('Diff_in_world_frame_of_Palm.csv',diff_in_world, delimiter=',')