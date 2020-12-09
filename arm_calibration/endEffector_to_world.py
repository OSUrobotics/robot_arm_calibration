#! usr/bin/env python3

# takes in the calibrated translation and rotation matrix with the recorded transform from the arm and gives the
# location of the end effector in the world frame.

import numpy as np
import csv
import sys
import os


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


with open(directory + '/Test_11_9/TransformMatrix_4.0.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            tran_mat[j][i] = float(col)


########################################################################################################################
# Calculate the end effector location in the world frame
########################################################################################################################

loc = rotation_mat @ translation_mat @ tran_mat @ np.transpose([0, 0, 0, 1])

print(loc)

