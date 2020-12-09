# robot_arm_calibration

**Software:** ROS and PyCharm

**Programming language:** Python

**Hardware:** Kinova and webcam

The main purpose of this experiment is to use the calibrated arm and camera to find the error reported by the arm and the camera "seeing." Then, adjust the joint angle of the arm to resolve this error. During the experiment, we used overhead cameras and ArUco markers to determine the angle of finger rotation. Then use MoveIt/Rviz to visualize the data collected from the arm and camera. Finally, we will determine the error between the physical arm and camera output positions of the joint angles. Using the error, we will implement an optimization method to correct the transformation matrices to confidently output an accurate position of the Kinova arm.
