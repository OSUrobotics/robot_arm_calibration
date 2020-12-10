import cv2
from cv2 import aruco
import numpy as np
import math
import glob

# calculate the orientation of ArUco Markers
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# Read the location and orientation of the ArUco Markers
def read_inf(ids, i):
    # make a dictionary of the ArUco Markers
    id = [1, 14]
    location = ['palm', 'end_effector']
    dict={}
    for n in range(len(id)):
        dict[id[n]] = location[n]

    name = dict[ids[i, 0]]

    # --- 180 deg rotation matrix around the x axis
    R_flip = np.zeros((3, 3), dtype=np.float32)
    R_flip[0, 0] = 1.0
    R_flip[1, 1] = -1.0
    R_flip[2, 2] = -1.0

    # transformation matrix (from camera frame to world frame)
    #change unit from cm to m
    tranf = np.eye(4)
    tranf[1, 1] = -1
    tranf[0, 3] = 0.015
    tranf[2, 2] = -1
    tranf[1, 3] = 0.01
    tranf[2, 3] = 0.8

    # make the location matrix of ArUco Marker
    pos = np.insert(tvecs[i],[3], 100, axis=None) / 100
    # get the world frame location
    trans = np.dot(tranf, pos)
    # get the output of the location
    str_position = name + " Position x=%4.3f  y=%4.3f  z=%4.3f" % (
        trans[0], trans[1], trans[2])

    # -- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvecs[i])[0])
    R_tc = R_ct.T

    # create the rotation matrix of each ArUco Markers
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                               dtype=float)
    # store the rotation vectors
    rotation_matrix[:3, :3], A = cv2.Rodrigues(rvecs[i])

    # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

    str_attitude = name + " Euler Angle x=%4.1f  y=%4.1f  z=%4.1f" % (
        math.degrees(roll_marker), math.degrees(pitch_marker),
        math.degrees(yaw_marker))

    # draw axis
    aruco.drawAxis(img, camera_matrix, camera_distortion, rvecs[i], tvecs[i], 1.5)

    rotates = [math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)]

    return str_position, str_attitude, trans, rotates, rotation_matrix

def transform_3D(loc_e, loc_p, rot_matrix_p, rot_matrix_e):

    # # obtain the average location and the average rotation matrices of the end effector and palm
    EE_to_P_sum = np.zeros(4)
    P_to_world_sum = np.zeros(4)

    for i in range(len(rot_matrix_e)):

        # transform the rotation matrices from camera frame to world frame
        # create the transform matrix from camera frame to world frame
        camera_to_world_for_rot = np.eye(4)
        camera_to_world_for_rot[1, 1] = -1
        camera_to_world_for_rot[2, 2] = -1

        # obtain the rotation matrices of end effector
        rotation_matrix_e = camera_to_world_for_rot @ rot_matrix_e[i]
        # obtain the rotation matrices of palm
        rotation_matrix_p = camera_to_world_for_rot @ rot_matrix_p[i]

        # calculate the translation matrix from end effector frame to world frame
        EE_to_world_trans = np.eye(4)
        EE_to_world_trans[0, 3] = loc_e[i][0]
        EE_to_world_trans[1, 3] = loc_e[i][1]
        EE_to_world_trans[2, 3] = loc_e[i][2]

        # calculate the transform matrix from end effector frame to world frame
        EE_to_world = EE_to_world_trans @ rotation_matrix_e
        # calculate the transform matrix from world frame to end effector frame
        world_to_EE = np.linalg.inv(EE_to_world)

        # change the location to be the quaternion
        new_loc_e = np.append(loc_e[i], [1])

        # check the result of matrices
        print('End Effector location\n', new_loc_e)
        print('Trans from EE to world\n', EE_to_world @ [0,0,0,1])
        print('Trans from world to EE\n', world_to_EE @ new_loc_e)

        # create the translation matrix from palm frame to world frame
        P_to_world_trans = np.eye(4)
        P_to_world_trans[0, 3] = loc_p[i][0]
        P_to_world_trans[1, 3] = loc_p[i][1]
        P_to_world_trans[2, 3] = loc_p[i][2]

        # calculate the transform matrix from palm frame to world frame
        P_to_world = P_to_world_trans @ rotation_matrix_p
        # calculate the transform matrix from world frame to palm frame
        world_to_P = np.linalg.inv(P_to_world)

        # change the location to be the quaternion
        new_loc_p = np.append(loc_p[i], [1])

        # check the result of matrices
        print('Palm location\n', new_loc_p)
        print('Trans from Palm to world\n', P_to_world @ [0,0,0,1])
        print('Trans from world to Palm\n', world_to_P @ new_loc_p)

        # calculate the transform matrix from end effector to palm
        EE_to_P = world_to_P @ EE_to_world

        # sum the transform matrices together
        EE_to_P_sum = EE_to_P_sum + EE_to_P
        P_to_world_sum = P_to_world_sum + P_to_world
    # average the sum matrix to reduce error
    EE_to_P_avg = EE_to_P_sum / len(rot_matrix_e)
    P_to_world_avg = P_to_world_sum / len(rot_matrix_e)

    return EE_to_P_avg, P_to_world_avg

# the size of ArUco Markers
marker_size = 3.6  # cm

# Marker IDs
end_effector_id = 14
palm_id = 1

# Load the dictionary of the ArUco Markers
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
parameters = aruco.DetectorParameters_create()

# --- Get the camera calibration path
calib_path = ""
camera_matrix = np.load(calib_path + 'camera_mtx.npy')
camera_distortion = np.load(calib_path + 'dist_mtx.npy')

train_images = glob.glob('EE_Palm/*.jpg')

# create lists to store the locations and rotation matrices of the end effector and the palm
loc_e = []
loc_p = []
rot_matrix_e = []
rot_matrix_p = []

# obtain the number of input images
num_file = len(train_images)

for n in range(num_file):
    # input image
    img = cv2.imread(train_images[n])

    # Detect the markers.
    corners, ids, rejected = aruco.detectMarkers(image=img, dictionary=aruco_dict, parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    # setting the position of text box
    font = cv2.FONT_HERSHEY_PLAIN
    # get the rotation and translation vectors
    rvecs, tvecs, _objPonits = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    # identify each ArUco Marker
    for i in range(ids.size):

        # detect the ArUco Markers of the end effector and the palm
        if ids[i] == end_effector_id:

            [str_position, str_attitude, trans, rotates, rotation_matrix] = read_inf(ids, i)

            cv2.putText(img, str_position, (0, 25), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.putText(img, str_attitude, (0, 50), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

            # store locations of end effector
            loc_e.append(trans[0:3])
            # store rotation matrices of end effector
            rot_matrix_e.append(rotation_matrix.T)

        if ids[i] == palm_id:

            [str_position, str_attitude, trans, rotates, rotation_matrix] = read_inf(ids, i)

            cv2.putText(img, str_position, (0, 75), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.putText(img, str_attitude, (0, 100), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

            # store locations of palm
            loc_p.append(trans[0:3])
            # store rotation matrices of palm
            rot_matrix_p.append(rotation_matrix.T)

# transform these lists to array
loc_p = np.array(loc_p)
loc_e = np.array(loc_e)
rot_matrix_p = np.array(rot_matrix_p)
rot_matrix_e = np.array(rot_matrix_e)

# call transform_3D to calculate transform matrices
EE_to_P_avg, P_to_world_avg = transform_3D(loc_e, loc_p, rot_matrix_p, rot_matrix_e)

# save transform matrices
np.savetxt('EE_to_Palm.csv', EE_to_P_avg, delimiter=',')
np.savetxt('P_to_world.csv', P_to_world_avg, delimiter=',')



