import numpy as np
import math
from scipy.spatial.transform import Rotation as R

loc_e = [1, 3, 1, 1]
rot_e = [0, 0, 0]
loc_p = [2, 2, 3, 1]
rot_p = [0, 0, 0]

EE_to_world_trans = np.eye(4)
EE_to_world_trans[0, 3] = loc_e[0]
EE_to_world_trans[1, 3] = loc_e[1]
EE_to_world_trans[2, 3] = loc_e[2]

EE_to_world_rot_x = np.eye(4)
EE_to_world_rot_x[1, 1] = np.cos(-math.radians(rot_e[0]))
EE_to_world_rot_x[1, 2] = -np.sin(-math.radians(rot_e[0]))
EE_to_world_rot_x[2, 1] = np.sin(-math.radians(rot_e[0]))
EE_to_world_rot_x[2, 2] = np.cos(-math.radians(rot_e[0]))

EE_to_world_rot_y = np.eye(4)
EE_to_world_rot_y[0, 0] = np.cos(-math.radians(rot_e[1]))
EE_to_world_rot_y[0, 2] = np.sin(-math.radians(rot_e[1]))
EE_to_world_rot_y[2, 0] = -np.sin(-math.radians(rot_e[1]))
EE_to_world_rot_y[2, 2] = np.cos(-math.radians(rot_e[1]))

EE_to_world_rot_z = np.eye(4)
EE_to_world_rot_z[0, 0] = np.cos(-math.radians(rot_e[2]))
EE_to_world_rot_z[0, 1] = -np.sin(-math.radians(rot_e[2]))
EE_to_world_rot_z[1, 0] = np.sin(-math.radians(rot_e[2]))
EE_to_world_rot_z[1, 1] = np.cos(-math.radians(rot_e[2]))

EE_to_world_rot = EE_to_world_rot_z @ EE_to_world_rot_y @ EE_to_world_rot_x

# print('1', EE_to_world_rot)
# AAA = R.from_euler('xyz', np.array(rot_e), degrees=True)
# print('2', AAA.as_matrix())

EE_to_world = EE_to_world_trans @ EE_to_world_rot
world_to_EE = np.linalg.inv(EE_to_world)
print('EE in Aru: ', loc_e)
print('EE in W: ', EE_to_world @ [0,0,0,1])
# print('P in EE', world_to_EE @ loc_p)


P_to_world_trans = np.eye(4)
P_to_world_trans[0, 3] = loc_p[0]
P_to_world_trans[1, 3] = loc_p[1]
P_to_world_trans[2, 3] = loc_p[2]

P_to_world_rot_x = np.eye(4)
P_to_world_rot_x[1, 1] = np.cos(-math.radians(rot_p[0]))
P_to_world_rot_x[1, 2] = -np.sin(-math.radians(rot_p[0]))
P_to_world_rot_x[2, 1] = np.sin(-math.radians(rot_p[0]))
P_to_world_rot_x[2, 2] = np.cos(-math.radians(rot_p[0]))

P_to_world_rot_y = np.eye(4)
P_to_world_rot_y[0, 0] = np.cos(-math.radians(rot_p[1]))
P_to_world_rot_y[0, 2] = np.sin(-math.radians(rot_p[1]))
P_to_world_rot_y[2, 0] = -np.sin(-math.radians(rot_p[1]))
P_to_world_rot_y[2, 2] = np.cos(-math.radians(rot_p[1]))

P_to_world_rot_z = np.eye(4)
P_to_world_rot_z[0, 0] = np.cos(-math.radians(rot_p[2]))
P_to_world_rot_z[0, 1] = -np.sin(-math.radians(rot_p[2]))
P_to_world_rot_z[1, 0] = np.sin(-math.radians(rot_p[2]))
P_to_world_rot_z[1, 1] = np.cos(-math.radians(rot_p[2]))

P_to_world_rot= P_to_world_rot_x @ P_to_world_rot_y @ P_to_world_rot_z
P_to_world = P_to_world_trans @ P_to_world_rot
world_to_P = np.linalg.inv(P_to_world)

print('P in Aru: ', loc_p)

print('XXX:', world_to_P @ EE_to_world @ [0,0,0,1])

M = world_to_EE @ loc_p
EE_to_P_trans = np.eye(4)
EE_to_P_trans[0, 3] = M[0]
EE_to_P_trans[1, 3] = M[1]
EE_to_P_trans[2, 3] = M[2]
# EE_to_P_trans = np.linalg.inv(EE_to_P_trans)


R = np.array(rot_e) - np.array(rot_p)
EE_to_p_x = np.eye(4)
EE_to_p_x[1, 1] = np.cos(math.radians(R[0]))
EE_to_p_x[1, 2] = -np.sin(math.radians(R[0]))
EE_to_p_x[2, 1] = np.sin(math.radians(R[0]))
EE_to_p_x[2, 2] = np.cos(math.radians(R[0]))

EE_to_p_y = np.eye(4)
EE_to_p_y[0, 0] = np.cos(math.radians(R[1]))
EE_to_p_y[0, 2] = np.sin(math.radians(R[1]))
EE_to_p_y[2, 0] = -np.sin(math.radians(R[1]))
EE_to_p_y[2, 2] = np.cos(math.radians(R[1]))

EE_to_p_z = np.eye(4)
EE_to_p_z[0, 0] = np.cos(math.radians(R[2]))
EE_to_p_z[0, 1] = -np.sin(math.radians(R[2]))
EE_to_p_z[1, 0] = np.sin(math.radians(R[2]))
EE_to_p_z[1, 1] = np.cos(math.radians(R[2]))

EE_to_p_R = EE_to_p_x @ EE_to_p_y @ EE_to_p_z

EE_to_p = EE_to_P_trans @ EE_to_p_R

print(EE_to_p @ loc_e)
print(EE_to_p @ [1, 1, 1, 1])

diff = np.array(loc_p) - np.array(loc_e)
print("diff" , diff)
EE_to_P_trans = np.eye(4)
EE_to_P_trans[0, 3] = diff[0]
EE_to_P_trans[1, 3] = diff[1]
EE_to_P_trans[2, 3] = diff[2]

R = np.array(rot_p) - np.array(rot_e)
EE_to_p_x = np.eye(4)
EE_to_p_x[1, 1] = np.cos(math.radians(R[0]))
EE_to_p_x[1, 2] = -np.sin(math.radians(R[0]))
EE_to_p_x[2, 1] = np.sin(math.radians(R[0]))
EE_to_p_x[2, 2] = np.cos(math.radians(R[0]))

EE_to_p_y = np.eye(4)
EE_to_p_y[0, 0] = np.cos(math.radians(R[1]))
EE_to_p_y[0, 2] = np.sin(math.radians(R[1]))
EE_to_p_y[2, 0] = -np.sin(math.radians(R[1]))
EE_to_p_y[2, 2] = np.cos(math.radians(R[1]))

EE_to_p_z = np.eye(4)
EE_to_p_z[0, 0] = np.cos(math.radians(R[2]))
EE_to_p_z[0, 1] = -np.sin(math.radians(R[2]))
EE_to_p_z[1, 0] = np.sin(math.radians(R[2]))
EE_to_p_z[1, 1] = np.cos(math.radians(R[2]))

EE_to_p_R = EE_to_p_z @ EE_to_p_y @ EE_to_p_x

M = EE_to_p_R @ EE_to_P_trans

print('T * ee:', EE_to_P_trans @ loc_e)
print('M * ee:',P_to_world @ M @ world_to_EE @ loc_e)




# A = np.eye(4)
# A[0, 3] = 6**(1/2)*np.cos(math.radians(R[0]))
# A[1, 3] = 6**(1/2)*np.cos(math.radians(R[1]))
# A[2, 3] = 6**(1/2)*np.cos(math.radians(R[2]))
# print('A:\n', A)
# print('EE_to_world:\n', EE_to_world)
# print(EE_to_world @ A @ [0,0,0,1])