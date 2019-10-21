# On-Using-Inertial-Measurement-Units-for-Solving-the-Direct-Kinematics-Problem-of-Parallel-Mechanisms

The video of the 3-RPR parallel mechanism can be found [here]. The video can be watched, for example, by downloading the entired project or by using this [download link].

[here]:https://github.com/stefanschulz85/On-Using-Inertial-Measurement-Units-for-Solving-the-Direct-Kinematics-Problem-of-Parallel-Mechanisms/blob/master/Video.mp4
[download link]:https://github.com/stefanschulz85/On-Using-Inertial-Measurement-Units-for-Solving-the-Direct-Kinematics-Problem-of-Parallel-Mechanisms/archive/master.zip
______________________________________________________________________________________________________________________

The direct kinematics problem of parallel mechanisms is the problem of finding the actual pose of the moveable manipulator platform with respect to the fixed base platform from the active joints' coordinates.
Unlike for inverse kinematics, in general, there is no unique solution for this problem for parallel mechanisms.
We investigate the accuracy and the computational efficiency of an IMU-based approach for solving the direct kinematics problem of parallel mechanisms with length-variable linear actuators under dynamic conditions. 
By avoiding to measure the linear actuators' lengths and by using orientations instead, a comprehensive, low-cost sensor structure can be obtained that provides a unique solution for the direct kinematics problem.  
As a representative example, we apply our approach to the planar 3-RPR parallel mechanism, where \underline{P} denotes active prismatic joints and R denotes passive revolute joints, and investigate the achievable accuracy and robustness on a specially designed experimental device. In this context, we also investigate the effect of sensor fusion on the achievable accuracy.
Finally, we compare our results with those obtained from linear actuators' lengths when the Newton-Raphson algorithm is used to compute the manipulator platform's pose iteratively.

<img src="https://github.com/stefanschulz85/On-Using-Inertial-Measurement-Units-for-Solving-the-Direct-Kinematics-Problem-of-Parallel-Mechanisms/Prototype.png" width="478" height="286" title="Experimental prototype of the general planar 3-RPR parallel mechanism with IMUs mounted on the linear actuators and an Arduino Uno with a display integrated in the base to calculate and show the manipulator platform's pose.">

Figure: Experimental prototype of the general planar 3-RPR parallel mechanism with IMUs mounted on the linear actuators and an Arduino Mega with a display integrated in the base to calculate and show the manipulator platform's pose.

<img src="https://github.com/stefanschulz85/Assembly-Modes-of-a-3-RPR-parallel-Mechanism-when-Using-the-Linear-Actuators-Orientations/blob/master/pictures/Assembly_Modes_A.png" width="280" height="692" title="The two assembly modes (shown in blue and red) for the manipulator platform of the general planar 3-RPR parallel mechanism when using the linear actuators’ orientations.">
______________________________________________________________________________________________________________________

The project currently contains the following folders:
- pictures
- video
