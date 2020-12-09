# Generate a Calibrated Transform Matrix for the Kinova Arm

The code in this folder will use the arm being manually moved to each corner, a known location, and an initial transform matrix from the arm base to the world frame.
Once this calibrated transform matrix is developed then the end effector is able to be placed in the world/camera frame and a location in the world/camera frame can
placed in the robots base frame.

1. Use Nuha's guide to get the ROS workspace for the Kinova arm working on your computer.
2. Add the following python programs to kinova_ws/src/kinova-ros/kinova_scrpits/src.
* transformMatrix.py, arm_calibration.py and recordAngles.py
3. enter the following into the terminal:
* cd ~/kinovs_ws
* catkin_make
* source devel/setup.bash
4. Plug the USB from the Kinova into the computer, and turn on the Kinova.
5. Press and hold the home button on the arms controller until the arm moves to the home position.
6. On the computer open 3 terminals, navigating to /kinova_ws and run the following in each:
* source devel/setup.bash
7. In the first terminal run the following:
*roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
8. The hand should now be open place the 3D printed object over the palm pad of the end effector (the end of the object marks the end effector for the arm)
9.  run the following in the second terminal:
* rosrun kinova_scripts transformMatrix.py <#>
  * <#>: initial location is 0, front left is 1, front right is 2, back right is 3, back left is 4, center is 5
  * This program will record the transform matrix from the arm's end effector to the arms base frame into a csv file
10. Manually move the arm so that the end effector object is directly over the aruco markers in each corner, once at the corner run the command shown above changing
the number for teh respected location, as listed.
11. Once the matrices are recorded for each corner the needed data for calibration is done.
12. Open arm_calibration.py and change the initial transform matrix(~line 19) with the measurements made from tehe center aruco marker to the arm's base.
13. Modife the file location (~line 36) to point to the folder that has the transformMatrices (csv files)
14. Once these modifications are done run the script, which will produce a TranslationMatrix.csv and RotationMatrix.csv.
* These files can be used to create the transform matrix that goes between the arm base and world/camera frame.



## Other Programs

* recordAngles.py
  * Records all of the joint angles for the arm into a csv while using it with ROS. Used with optimization code.
  * rosrun kinova_scripts recordAngles.py <#>
    * <#> Enter a number to distingush between different csv files
* endEFfector_to_world.py
  * Uses the calibrated transform matrix to and the recorded transform matrix from the arm's end effector to the base to put the 
  end effectors location into the world frame.
* world_to_armBase.py
  * Puts a location from the world frame into the arm's base frame.
  
## Other Info

Currently the Z axis is not calibrated an update to the testbed top is needed to allow for full calibration.
