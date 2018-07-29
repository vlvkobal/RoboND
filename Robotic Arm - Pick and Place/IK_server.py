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
from time import time
import angles
import gazebo_msgs.srv

# Set this to True for error evaluation
evaluate_errors = False
errors_file_name = 'IK_FK_errors.csv'

print("Preparing Inverse Kinematics matrices...")
start_time = time()

# Create symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, dG = symbols('d1:7, dG')
theta1, theta2, theta3, theta4, theta5, theta6, thetaG = symbols('theta1:7, thetaG')
roll, pitch, yaw = symbols('roll, pitch, yaw')

# Create DH parameters
DH_param = {alpha0:     0, a0:      0, d1:   0.75,
            alpha1: -pi/2, a1:   0.35, d2:      0, theta2: theta2 - pi/2,
            alpha2:     0, a2:   1.25, d3:      0,
            alpha3: -pi/2, a3: -0.054, d4:    1.5,
            alpha4:  pi/2, a4:      0, d5:      0,
            alpha5: -pi/2, a5:      0, d6:      0,
            alpha6:     0, a6:      0, dG:  0.303, thetaG: 0}

# Create transformation matrices

def individual_DH_transform(alpha, a, d, theta):
    return Matrix([[           cos(theta),           -sin(theta),           0,             a],
                   [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                    0,                     0,           0,             1]])

T0_1 = individual_DH_transform(alpha0, a0, d1, theta1).subs(DH_param)
T1_2 = individual_DH_transform(alpha1, a1, d2, theta2).subs(DH_param)
T2_3 = individual_DH_transform(alpha2, a2, d3, theta3).subs(DH_param)
T0_3 = T0_1 * T1_2 * T2_3

TG_Cz = Matrix([[cos(pi), -sin(pi), 0, 0],
                [sin(pi),  cos(pi), 0, 0],
                [      0,        0, 1, 0],
                [      0,        0, 0, 1]])

TG_Cy = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                [          0, 1,          0, 0],
                [-sin(-pi/2), 0, cos(-pi/2), 0],
                [          0, 0,          0, 1]])

# Calculate rotation matrix for given orientation

R_x = Matrix([[ 1,         0,          0],
              [ 0, cos(roll), -sin(roll)],
              [ 0, sin(roll),  cos(roll)]])

R_y = Matrix([[ cos(pitch), 0,  sin(pitch)],
              [          0, 1,           0],
              [-sin(pitch), 0,  cos(pitch)]])

R_z = Matrix([[ cos(yaw), -sin(yaw), 0],
              [ sin(yaw),  cos(yaw), 0],
              [        0,         0, 1]])

Rrpy = R_z * R_y * R_x

# Prepare inverted matrices for IK
R0_3T = simplify(T0_3[0:3, 0:3].transpose())
RG_CT = (TG_Cz[0:3, 0:3] * TG_Cy[0:3, 0:3]).transpose()

print ("Inverse Kinematics matrices were prepared in %04.4f seconds" % (time()-start_time))

class FK_matrices(object):
    """prepares FK matrices for error evaluation"""
    def __init__(self, DH_param):
        print("Preparing Forward Kinematics matrices...")
        start_time = time()
        T3_4 = individual_DH_transform(alpha3, a3, d4, theta4).subs(DH_param)
        T4_5 = individual_DH_transform(alpha4, a4, d5, theta5).subs(DH_param)
        T5_6 = individual_DH_transform(alpha5, a5, d6, theta6).subs(DH_param)
        T6_G = individual_DH_transform(alpha6, a6, dG, thetaG).subs(DH_param)
        T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G * TG_Cz * TG_Cy
        self.FK_ee = simplify(T0_G[0:3, 3])
        print ("Forward Kinematics matrices were prepared in %04.4f seconds" \
               % (time()-start_time))

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        if evaluate_errors:
            with open(errors_file_name, 'a') as errors_file:
                errors_file.write('New path received\n')

        # Initialize service response

        # Get current joint angles
        try:
            get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', \
                                                      gazebo_msgs.srv.GetJointProperties)
            p_theta4 = get_joint_properties('joint_4').position[0]
            p_theta5 = get_joint_properties('joint_5').position[0]
            p_theta6 = get_joint_properties('joint_6').position[0]
        except rospy.ServiceException, e:
            p_theta4 = 0
            p_theta5 = 0
            p_theta6 = 0

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

            (v_roll, v_pitch, v_yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here

            start_time = time()

            # Calculate wrist center position
            ee = Matrix([[px], [py], [pz]])
            v_Rrpy = Rrpy.subs({roll: v_roll, pitch: v_pitch, yaw: v_yaw})
            wc = ee - 0.303 * (v_Rrpy * Matrix([[1], [0], [0]]))

            # Calculate theta1
            v_theta1 = atan2(wc[1], wc[0])
            DH_param[theta1] = v_theta1
            
            # Calculate theta2
            length_O2_WCxy = sqrt(wc[0]**2 + wc[1]**2) - DH_param[a1]
            length_O2_WCz = wc[2] - DH_param[d1]
            length_O2_WC = sqrt(length_O2_WCxy**2 + length_O2_WCz**2)
            length_O3_WC = sqrt(DH_param[d4]**2 + DH_param[a3]**2)
            beta = atan2(length_O2_WCz, length_O2_WCxy)
            beta2 = acos((DH_param[a2]**2 + length_O2_WC**2 - length_O3_WC**2) \
                         / (2 * DH_param[a2] * length_O2_WC))
            v_theta2 = pi/2 - (beta2 + beta)
            DH_param[theta2] = v_theta2

            # Calculate theta3
            gamma = acos((DH_param[a2]**2 + length_O3_WC**2 - length_O2_WC**2) \
                         / (2 * DH_param[a2] * length_O3_WC))
            gamma2 = pi/2 - atan2(DH_param[a3], DH_param[d4])
            v_theta3 = pi - (gamma + gamma2)
            DH_param[theta3] = v_theta3

            # Calculate wrist rotation matrix
            R3_6 = R0_3T.subs(DH_param) * v_Rrpy * RG_CT

            # Calculate theta5,6,7
            v_theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            v2_theta4 = atan2(-R3_6[2,2], R3_6[0,2])
            v_theta5 = acos(R3_6[1,2])
            v_theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            v2_theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            # Decrease number of wrist flippings
            longt_wrist_rotation_limit = 350 * pi / 180
            ang_dist_theta4 = angles.shortest_angular_distance(p_theta4, v_theta4)
            ang_dist2_theta4 = angles.shortest_angular_distance(p_theta4, v2_theta4)
            if abs(p_theta4 + ang_dist2_theta4) < longt_wrist_rotation_limit \
               and (abs(ang_dist2_theta4) + abs(-v_theta5 - p_theta5)) \
                    < (abs(ang_dist_theta4) + abs(v_theta5 - p_theta5)):
                v_theta4 = p_theta4 + ang_dist2_theta4
                v_theta5 = -v_theta5
                v_theta6 = v2_theta6
            else:
                v_theta4 = p_theta4 + ang_dist_theta4
            ang_dist_theta6 = angles.shortest_angular_distance(p_theta6, v_theta6)
            if abs(p_theta6 + ang_dist_theta6) < longt_wrist_rotation_limit:
                v_theta6 = p_theta6 + ang_dist_theta6
            # Memorize theta4,5,6 for the next cycle
            p_theta4 = v_theta4
            p_theta5 = v_theta5
            p_theta6 = v_theta6

            # Evaluate error for end effector position
            if evaluate_errors:
                DH_param[theta4] = v_theta4
                DH_param[theta5] = v_theta5
                DH_param[theta6] = v_theta6
                calculated_ee = FK.FK_ee.subs(DH_param)
                with open(errors_file_name, 'a') as errors_file:
                    errors_file.write(str(sqrt(abs(px - calculated_ee[0, 0])**2 \
                                          + abs(py - calculated_ee[1, 0])**2 \
                                          + abs(pz - calculated_ee[2, 0])**2)) + '\n')

            print ("%i: pose calculation time: %04.4f seconds" % (x, (time()-start_time)))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [v_theta1, v_theta2, v_theta3, v_theta4, v_theta5, v_theta6]
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
    if evaluate_errors:
        FK = FK_matrices(DH_param)
    IK_server()
