# ANFIS-model-for-Inverse-kinematics-Problem
This repository contains a Matlab code to solve the inverse kinematic solution of a 3R planar manipulator.

Primary feature of an ANFIS system is that it performs automatic tuning of the membershipfunctions of the fussy sets to better learn the problem at hand. In trying to and the solution to the Inverse Kinematics problem using ANFIS, we needed to generate the data with inputs and outputs. 

**NOTE:According to the link lengths 10,7 and 5 for link1, link2 and link3 respectively and the joint angle constraints of 0 <= theta <= pi/2 for all the joints, the end effector positions were formulated using the forward kinematics equations.

Using linspace the joint angles were selected equally spaced in the given joint restrictions and were then used to and out the end effector’s positions X and Y along with the end effector’s angle with respect to the origin. The validation set was generated similarly but by selecting the joint angles randomly withing the constraints.

Each joint of the robot manipulator is to be trained individually. Therefore, 3 training datasets were created with the inputs as the X,Y coordinates, end effector angle, and its corresponding joint angle. This data was fed to the respective ANFIS networks that were initiated with 5 triangular Gaussian membership functions and were limited to 15 epochs. The shape of the membership function didn’t seem to change the accuracy of the trained network. 5 distinctions were chosen as it was a good balance between the computation speed and the accuracy.

The iterations were limited to 15 epochs as the error decline rate was significantly lower after 15 epochs and was therefore only consuming the extra time without much progress.
These ANFIS were trained successively trained and the results were analysed with the help of the root mean squared error (RMSE) of both the training and the validation results. These are represented as shown in the figures (1,2,3). The results represented show that the RMSE value for all the joint angle are in the same order of magnitude 10^−2 and their corresponding MSE would be in the order of 10^−4, with the error for the second joint angle being slightly higher than the other two.
