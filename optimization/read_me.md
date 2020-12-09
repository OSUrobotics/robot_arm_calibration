## Optimization: Gradient Descent

Gradient descent uses the gradient to determine the direction of deepest ascent to minimize the loss between the current position of the arm end effector (from the physical robot) and target end effector position (from the arUco marker output). To use this optimization technique, we adjust the value of each joint angle by a certain step size, then determine the error given the updated arm end effector location. Each step that is taken slightly adjusts the coordinate position of the end effector towards the target position, minimizing the error. The arm position is continually updated until the change in cost between the previous end effector location and updated end effector location is acceptable within a certain threshold.

### The code depends on the following files present:
arm_calibration.py: Calculates the arm position to be within the world frame
endEffector_to_world.py: Calculate the end effector location in the world frame
arm_cal/ : Contains the physical joint angles ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7") and transformation matrices

### To run the code:
This code has been run and tested using Python 3 and above.
Make sure all required python libraries are installed (numpy, matplotlib)

**Run the following command:**
```python gradient_descent.py```

**Input:**
Physical joint angles and transformation matrices within arm_cal/
Target end effector location based on the arUco markers

**Output:**
Optimized joint angle values for each joint and final end effector coordinates
