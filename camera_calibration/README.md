# Camera Calibration
The code in this folder is used to reduce the distortion of the web-camera and read the positions and orientations of the ArUco Markers on the robot hand. Once we obtained the camera matrix and the distortion from "camera calibration.py", we can obtain the positions and orientations of the robot arm by using OpenCV to read the ArUco Markers. Then, we will calculate the error of the robot frame to the world frame and optimize it. 
* Software: Python

## camera calibration.py
1. Using the web-camera to take 30-50 checkerboard images to be the input images of "camera calibration.py". Our checkerboard images are in the "image_processing folder".
2. Run "camera calibration.py" and obtain "camera_mtx.npy" and "dist_mtx.npy".
3. One sample result image of "camera calibration.py" is named "calibresult.png".
* This code refers to Nuha's code and OpenCV's tutorial (https://docs.opencv.org/master/d9/d0c/group__calib3d.html#exercises)

## identify aruco markers in image.py
1. Using the web-camera to take the input images of "identify aruco markers in image.py".
2. Run "identify aruco markers in image.py". The code needs "camera_mtx.npy" and "dist_mtx.npy" to be the camera parameters.
3. The results of "identify aruco markers in image.py" are in the "result folder". The data file contains the positions and orientations of each ArUco Marker. The resulting image shows the information in the top left corner of the input image.
4. Besides that, "identify aruco markers in image.py" will create "tranf_world_to_camera.csv" which content is the transform matrix from the world frame to the camera frame.
* This code refers to Nuha's code and Tiziano Fiorenzani's code (https://github.com/tizianofiorenzani/how_do_drones_work/tree/master/opencv).

## calculate the transform matrices from end effector to palm
1. Using the web-camera to take the input images of "EE_Palm_Transform.py". (Our input images are in "EE_Palm folder")
2. Run "EE_Palm_Transform.py" to obtain the transform matrix from the end effector to palm in the end effector frame and the transform matrix from palm to end effector in the world frame.
3. This code creates two csv files named "EE_to_Palm.csv" and "P_to_world.csv". Move these two files into "Error_calculation folder"

## error calculation
1. Enter the "Error_calculation folder" and copy the specific data file from "result folder" and paste it into this folder. (We use data_file_9)
2. Run the codes "arm_calibration.py" and "world_to_armBase.py" which come from Josh to obtain the rotation matrix and the translation matrix.
3. Run the code "endEffector_to_world.py" to calculate the difference of the Palm's locations which come from ArUco Marker detection and the transform matrices calculation.
4. The code creates a result file names "Diff_in_world_frame_of_Palm.csv"

## file description
1. Inputs
* image_processing folder: The folder of the processing images for "camera calculation.py"
* EE_Palm folder: The folder of the input images for "EE_Palm_Transform.py"
* Validation folder: The folder of the test images of each script.
* 6.jpg\9.jpg\10.jpg\2020-12-01-133130.jpg\2020-12-01-133142.jpg\2020-12-01-133201.jpg\2020-12-01-140632.jpg: The input images of "identify aruco markers in image.py"

2. Algorithms
* camera calculation.py: The algorithm of camera calibration.
* identify aruco markers in image.py: The algorithm to read the information of the ArUco Markers
* EE_Palm_Transform.py: The algorithm of obtaining the transform matrics from end effector to palm
* Error_calculation folder\ endEffector_to_world.py: The algorithm of calculating the difference of the palm's locations which come from ArUco Markers and transform matrices.

3. Result
* camera_mtx.npy: The file which stored the camera matrix of the web-camera.
* dist_mtx.npy: The file which stored the distortion coefficients of the web-camera.
* tranf_world_to_camera.csv: The result of "identify aruco markers in image.py" which contains the transform matrix from the world frame to the camera frame.
* result folder: The folder of result data files and images of "identify aruco markers in image.py"
* calibresult.png: The resulting image of "camera calculation.py"
* EE_to_Palm.csv: The result of "EE_Palm_Transform.py" which contains the transform matrix from the end effector to palm in the end effector frame
* P_to_world.csv: The result of "EE_Palm_Transform.py" which contains the transform matrix from palm frame to world frame
