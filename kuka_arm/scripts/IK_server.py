#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


r, p, y = symbols('r p y')

#build Rrpy
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

#build DH transformation unit
def dh_trans_unit(p, a, d, q): #p as alpha, q as theta
    ret = Matrix([
        [       cos(q),       -sin(q),      0,          a],
        [sin(q)*cos(p), cos(q)*cos(p), -sin(p), -sin(p)*d],
        [sin(q)*sin(p), cos(q)*sin(p),  cos(p),  cos(p)*d],
        [            0,             0,       0,         1]])
    
    return ret


q1, q2, q3, q4, q5, q6 = symbols('q1:7')

#build T0_G
def dh_trans_build():

    # DH Table from urdf
    # alpha[i-1], a[i-1], d[i], theta[i]
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
    return ret

#build inv(R0_3)
def dh_build_invR0_3():
    R0_3 = Matrix([
        [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
        [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
        [        cos(q2 + q3),        -sin(q2 + q3),        0]])
    return simplify(R0_3**-1)

G_invR0_3 = dh_build_invR0_3()

#build R3_6
def dh_build_R3_6(Rrpy):
    # R0_6 = Rrpy * R_corr (R_corr is from DH to URDF
    R_z = Matrix([[cos(pi), -sin(pi), 0],
                  [sin(pi), cos(pi),  0],
                  [0,       0,        1]])
    R_y = Matrix([[cos(-pi/2), 	0,  sin(-pi/2)],
                  [0,      	1,  0],
                  [-sin(-pi/2), 0,  cos(-pi/2)]])

    R_corr = R_z * R_y
    R3_6 = G_invR0_3 * Rrpy * R_corr

    return simplify(R3_6)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            #0.303 = link_6 + gripper_link
            Rrpy = G_Rrpy.evalf(subs={r: roll, p: pitch, y: yaw})
            wx = px - 0.303 * Rrpy[0, 0]
            wy = py - 0.303 * Rrpy[1, 0]
            wz = pz - 0.303 * Rrpy[2, 0]

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

            #Calculate q4/5/6
            R3_6 = dh_build_R3_6(Rrpy).evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[1, 0]**2 + R3_6[1, 1]**2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
