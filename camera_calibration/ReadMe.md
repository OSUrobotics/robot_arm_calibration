# Camera Calibration
The code in this folder is used to reduce the distortion of the web-camera and read the positions and orientations of the ArUco Markers on the robot hand. Once we obtained the camera matrix and the distortion from "camera calibration.py", we can obtain the positions and orientations of the robot arm by using OpenCV to read the ArUco Markers.
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

