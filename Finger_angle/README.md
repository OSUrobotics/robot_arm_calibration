# Calculate the finger angle
In this experiment, two methods are used to calculate the angle of the finger. The first method is based on the rotation angle of the ArUco marker. The second method is based on the position of the ArUco marker.

It should be noted that when the ArUco marker reads the rotation angle, the returned angle range is from negative 180 degrees to positive 180 degrees. Also, the angle is negative when rotating clockwise, and the angle is positive when rotating counterclockwise. If the angle is more than 180 degrees clockwise, the angle will be corrected to the counterclockwise rotation's corresponding angle.

```Finger_angle.py```: The main code for finger angle calculation  
```data_file_6.csv```: Position and angle value in image 6  
```data_file_10.csv```: Position and angle value in image 10  
```physical model measurement.jpg```: The result of physical model measurement  

## Based on the rotation angle of the ArUco marker
The function of this method:```read_aruco_angle``` in ```Finger_angle.py```

In this method, only need to read five ArUco marker rotation angles: palm, finger_1_proxmal, finger_1_distal, finger_2_proxmal, finger_2_distal. Then, calculate the angle difference between each joint and palm to get the finger's opening and closing angle.

It is worth noting that there is a 180-degree difference between the two ArUco markers of finger_2 and the ArUco marker of palm in the initial state, so the ArUco marker angle of finger_2 needs to be reinitialized.

## Based on the position of the ArUco marker
The function of this method:```read_aruco_position``` in ```Finger_angle.py```

In this method, you only need to read the positions of the five ArUco markers of palm, finger_1_proxmal, finger_1_distal, finger_2_proxmal, and finger_2_distal, and then obtain the angle of the finger according to the relationship between the positions.

The specific details of the calculation:
* First move the palm position to the origin position, which is the position of the center point ArUco marker, and move the other fingers to the corresponding positions.
* Rotate the entire palm and fingers to the initial state, and the palm's rotation angle determines the rotation angle. It is worth noting that the angle of rotation is positive or negative, so it is necessary to determine the positive or negative of the theta angle in the rotation matrix.
* Calculate the center position of the finger. The method used here is that the distance between the center position of the finger and the ArUco marker and joints remains unchanged.
* Calculate the projection of the finger center in the x and y directions as dx and dy. Then the rotation angle of the finger can be known through the trigonometric function relationship.

## Output example
Test dataset from ```data_file_10.csv```
* Finger_1_proxmal_angle:
  * 78.32 (angle from method one)
  * 89.09 (angle from method two)
  * 83.71 (average angle)
* Finger_1_distal_angle:
  * 145.6 (angle from method one)
  * 122.2 (angle from method two)
  * 133.9 (average angle)
* Finger_2_proxmal_angle:
  * 75.31 (angle from method one)
  * 108.4 (angle from method two)
  * 91.86 (average angle)
* Finger_2_distal_angle:
  * 143.3 (angle from method one)
  * 175.4 (angle from method two)
  * 159.4 (average angle)

## Note
The accuracy of method two is lower than method one because the error of method one only from rotation angle reading. For the method two, the error from the physical model measurement, and position reading.
