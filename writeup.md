## Project: Kinematics Pick & Place


---


**Steps to complete the project:**  


[//]: # (Image References)

[image1]: ./misc_images/dh.png
[image2]: ./misc_images/q123.png
[image3]: ./misc_images/demo1.png
[image4]: ./misc_images/demo2.png
[image5]: ./misc_images/demo3.png
[image6]: ./misc_images/demo4.png
[image7]: ./misc_images/demo5.png

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

Alpha(i-1) is the angle from Z(i-1) to Z(i) based on X(i-1).

A(i-1) is the distance from Z(i-1) to Z(i) based on X(i-1).

D(i) is the distance from X(i-1) to X(i) based on Z(i).

Theta(i) is the angle from  X(i-1) to X(i) based on Z(i).


Note that all the angles need be calculated under the right hand rule, including the joint angles, which is based on Z(i).

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

1) DH Parameters was built as the following table.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

2) I use two steps to build the transformation matrix from Frame_0 to Frame_EE. The first step is build single transformation matrix from frame i-1 to frame i.

	def dh_trans_unit(p, a, d, q): #p as alpha, q as theta
	    ret = Matrix([
	        [       cos(q),       -sin(q),      0,          a], 
	        [sin(q)*cos(p), cos(q)*cos(p), -sin(p), -sin(p)*d],
	        [sin(q)*sin(p), cos(q)*sin(p),  cos(p),  cos(p)*d],
	        [            0,             0,       0,         1]])
	    
	    return ret 
	    
Then the follow code snippet was used to build the transformation matrix from Frame_0 to Frame_EE.

    dh_params = Matrix([
            [0,     0,      0.75,   q1],
            [-pi/2, 0.35,   0,      q2-pi/2],
            [0,     1.25,   0,      q3],
            [-pi/2, -0.054, 1.5,    q4],
            [pi/2, 0,       0,      q5],
            [-pi/2, 0,      0,      q6],
            [0,     0,      0.303,  0]])

    ret = eye(4)
    for i in range(0, dh_params.rows):
        trans_unit = dh_trans_unit(dh_params[i,0], dh_params[i,1], dh_params[i,2], dh_params[i,3])
        ret = ret * trans_unit

    ret = simplify(ret)
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

1 First, since we have the case of a spherical wrist involving joints 4,5,6, the position of wrist center is governed by the first three joints, it is possible to decoule the inverse kinematics problem into position and orientation problems. 

I use EE points and Rrpy from simulater to calculate WC coordinate with the following code snippet.

		def rotation_rpy():
		    R_x = Matrix([[1,       0,       0],
		                  [0,  cos(r), -sin(r)],
		                  [0,  sin(r),  cos(r)]])
		
		    R_y = Matrix([[cos(p), 0,  sin(p)],
		                  [0,      1,       0],
		                  [-sin(p), 0, cos(p)]])
		
		    R_z = Matrix([[cos(y),  -sin(y), 0],
		                  [sin(y),   cos(y), 0],
		                  [     0,        0, 1]])
		    return simplify(R_z*R_y*R_x)
		
		G_Rrpy = rotation_rpy()

Rrpy is the rotation matrix from Frame_0 to Frame_EE in URDF space, using external rotation by x/y/z. The angle roll, pitch, yaw is from simulater. 

px, py, pz is the EE coordinate in Frame_0, what we want to know is the wrist center coordinate in Frame_0 space. Because in Frame_EE space, both EE and WC are on Z axis, column 2 of Rrpy is the vector along Z axis, we can calculate WC with the above code snippet.

       #0.303 = link_6 + gripper_link
       Rrpy = G_Rrpy.evalf(subs={r: roll, p: pitch, y: yaw})
       wx = px - 0.303 * Rrpy[0, 0]
       wy = py - 0.303 * Rrpy[1, 0]
       wz = pz - 0.303 * Rrpy[2, 0]
       


2 Given WC, I use the following image to calculate theta1, theta2, and theta3.
![alt text][image2]

In the above picture, let's say segment from O2 to O3 is c, O2 to WC is b, O3 to WC is a; angle from b to c is ahpha, angle from c to a is beta. Easy to see that length of c is 1.25, length of a is ```sqrt(1.5*1.5+0.054*0.054)```, length of be is ```sqrt(pow((sqrt(wx*wx+wy*wy)-0.35), 2) + pow((wz-0.75), 2))```.

1) Theta(1) is quite simple, theta1 = atan2(wy, wx)  would be good.

2) Theta(2) is pi/2 - alpha - delta, alpha could be calculated with the help of cosine theorem ```alpha = acos((b*b + c*c - a*a) / (2*b*c))```, delta could be calculated with ```delta = atan2(wz-0.75, sqrt(wx*wx+wy*wy)-0.35)```.

2) Theta(3) is pi/2-beta-epsilon. beta could be calculated with the help of cosine theorem ```beta = acos((a*a + c*c - b*b) / (2*a*c))```, epsilon is ```epsilon = atan2(0.054, 1.5)```.

3 Then with theta1/2/3, I could calculate R3_6 with ```inv(R0_3) * Rrpy * R_corr```, and try to calculate theta4/5/6 base on the following transmation matrix.

	R3_6 = Matrix([
		[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
		[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
		[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]
	])

From the simplified R3_6 matrix, easy to get the following equations.

    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    theta5 = atan2(sqrt(R3_6[1, 0]**2 + R3_6[1, 1]**2), R3_6[1, 2])
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


1 The code is based on the equation R0_6 = Rrpy * R_corr, where R0_6 is the rotation matrix from frame 0 to frame 6 in the DH space. Rrpy is in the URDF space, R_corr rotate from DH space to URDF space.

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
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
          
      
Following is some images I captured when I was running the project.

1) Try to grasp the object.

![alt text][image3]

2) Retrive the object.

![alt text][image4]

3) Calculate inverse kinematics.

![alt text][image5]

4) Reach the can.

![alt text][image6]

5) Release the object.

![alt text][image7]


Improvement could be made for this project furture. The main issue is the time cost when calculating inverse kinematics. It cost me about 5 mins on my computer. Skills like using ```tranpose R0_3``` instead of ```inverse R0_3```, using numpy instead of sympy, using saving and loading matrix to avoid duplicate computing would improve the efficiency.
