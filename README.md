# On-Using-Inertial-Measurement-Units-for-Solving-the-Direct-Kinematics-Problem-of-Parallel-Mechanisms




______________________________________________________________________________________________________________________

The direct kinematics problem of parallel mechanisms is the problem of finding the actual pose of the moveable manipulator platform with respect to the fixed base platform from the active joints' coordinates.
Unlike for inverse kinematics, in general, there is no unique solution for this problem for parallel mechanisms.
We investigate the accuracy and the computational efficiency of an IMU-based approach for solving the direct kinematics problem of parallel mechanisms with length-variable linear actuators under dynamic conditions. 
By avoiding to measure the linear actuators' lengths and by using orientations instead, a comprehensive, low-cost sensor structure can be obtained that provides a unique solution for the direct kinematics problem.  
As a representative example, we apply our approach to the planar 3-RPR parallel mechanism, where \underline{P} denotes active prismatic joints and R denotes passive revolute joints, and investigate the achievable accuracy and robustness on a specially designed experimental device.
In this context, we also investigate the effect of sensor fusion on the achievable accuracy.
Finally, we compare our results with those obtained from linear actuators' lengths when the Newton-Raphson algorithm is used to compute the manipulator platform's pose iteratively.
