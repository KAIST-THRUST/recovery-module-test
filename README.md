# Arduino code for testing recovery module.

## PLEASE READ THIS BEFORE TESTING

1. Make sure all jumper wires are connected secuerly with Arduino.
2. Install MPU9250 with sensor's x-axis points to the nose.
3. Power up the board after the testing module is positioned up-right, i.e., the nose should point toward the sky.
4. Tilt the testing module and check if it works properly.

## HOW IT WORKS
The servo motor attached to pin 9 rotates by 90 degrees when the testing module is tilted more than 45 degrees from its initial attitude.
If the attitude returns back, the servo is rotated to the original position.

## File Description
recovery_test.ino: main program\
attitude.h: quaternion, dcm, and tilt check function implementations
