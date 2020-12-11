## Optimization: Gradient Descent

Gradient descent uses the gradient to determine the direction of deepest ascent to minimize the loss between the current position of the arm end effector (from the physical robot) and target end effector position (from the arUco marker output). To use this optimization technique, we adjust the value of each joint angle by a certain step size, then determine the error given the updated arm end effector location. Each step that is taken slightly adjusts the coordinate position of the end effector towards the target position, minimizing the error. The arm position is continually updated until the change in cost between the previous end effector location and updated end effector location is acceptable within a certain threshold.

### The code depends on the following files present:

final_test/ : Contains all needed transformation matrices and test data, including world transformations

final_test/test_data/ : Contains test images and transformation matrices for physical joint angles

### To run the code:

This code has been run and tested using Python 2

Make sure all required python libraries are installed (numpy, matplotlib)

**Run the following command along with the Kinova arm visualization code:**

```rosrun kinova_scripts gradient_descent_ROS.py```

**Input:**

Physical joint angles and transformation matrices

Target end effector location based on the arUco markers

**Output:**

Optimized joint angle values for each joint and final end effector coordinates
