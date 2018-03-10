## Project: Kinematics Pick & Place


---


**Steps to complete the project:**  


[//]: # (Image References)

[image1]: ./misc_images/dh.png
[image2]: ./misc_images/q123.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I use Denavit-Hartenberg (DH) parameters to do the forward and inverse kinematics for kuka_arm.
Following are the frames of every joint of kuka_arm.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

1 First, I use EE points and Rrpy from simulater to calculate WC coordinate.

2 Given WC, I use the following image to calculate theta1, theta2, and theta3.
![alt text][image2]

3 Then with the help of Euler angles, I calucate theta4, theta5 and theta6. 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


1 The code is based on the equation R0_6 * R_corr = Rrpy, where R0_6 is the rotation matrix from frame 0 to frame 6 in the DH space. Rrpy is in the URDF space, R_corr rotate from DH space to URDF space.

2 In my code, the inverse kinematics is decoupled into two parts, in the first part, I tred to calucate EE coordinate from EE coordinate and Rrpy.

            Rrpy = G_Rrpy.evalf(subs={r: roll, p: pitch, y: yaw})
            wx = px - 0.303 * Rrpy[0, 0]
            wy = py - 0.303 * Rrpy[1, 0]
            wz = pz - 0.303 * Rrpy[2, 0]


3 After that, I tried to calculate theta1, theta2 and theta3.

            ### Calculate q1/2/3
            theta1 = atan2(wy, wx) 

            a = sqrt(pow(0.054, 2) + pow(1.5, 2)) 
            b = sqrt(pow((sqrt(wx*wx+wy*wy)-0.35), 2) + pow((wz-0.75), 2)) 
            c = 1.25

            alpha = acos((b*b + c*c - a*a) / (2*b*c))
            beta = acos((a*a + c*c - b*b) / (2*a*c))

            delta = atan2(wz-0.75, sqrt(wx*wx+wy*wy)-0.35)
            theta2 = pi/2 - alpha - delta

            epsilon = atan2(0.054, 1.5)
            theta3 = pi/2 - beta - epsilon

4 In the end, I first calcuate ```R3_6=inv(R0_3) * Rrpy * R_corr```, then I calculate theta4, theta5 and theta6 as followed.

            #Calculate q4/5/6
            R3_6 = dh_build_R3_6(Rrpy).evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[1, 0]**2 + R3_6[1, 1]**2), R3_6[1, 2])
            theta6 = atan2(R3_6[1, 1], R3_6[1, 0])