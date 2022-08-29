# Gravity Balance for Walking Robot Leg
Midterm Project of Course 6614 ADV Topics in Robotics

## Project Presentation in Youtube
https://youtu.be/F7wg5U9hHzI

## The schematic diagram of the robot leg
Since the robot leg structure is a planar four-bar linkage and the axis of the hip pitch actuator and the axis of the knee actuator are collinear, we can achieve gravity balance with two springs as follows:

<p align="center">
    <img src="https://github.com/Qincheng-Sheng/Gravity_Balance_Robot_Leg/blob/main/pictures/structure.png" alt="system" width= "250">
</p>


## Robot Leg Rotation Simulation
In below figures, the red line represents the spring, the green line represents the rotating link, and the red dot means the joint that needs to be fixed on the robot body. Then, by doing simulation we find that different motion patterns can be realized.

Rotating joint 1 makes the upper leg swing, and rotating joint 2 makes the lower leg swing. If rotating these two joints at the same time would realize the actions of standing and squatting. Rotating joint 3 will make the robot legs swing left and right, so that the robot legs are able to turn left or turn right.

Hip pitch and knee Rotation. 
<p align="center">
        <img src="https://github.com/Qincheng-Sheng/Gravity_Balance_Robot_Leg/blob/main/pictures/hip_pitch.gif" alt="system" width= "350" />
        <img src="https://github.com/Qincheng-Sheng/Gravity_Balance_Robot_Leg/blob/main/pictures/knee.gif" alt="system" width= "350" />
</p>
 
Hip roll and Mixed Rotation.
<p align="center">
        <img src="https://github.com/Qincheng-Sheng/Gravity_Balance_Robot_Leg/blob/main/pictures/hip_roll.gif" alt="system" width= "350" />
        <img src="https://github.com/Qincheng-Sheng/Gravity_Balance_Robot_Leg/blob/main/pictures/mixed.gif" alt="system" width= "350" />
</p>
