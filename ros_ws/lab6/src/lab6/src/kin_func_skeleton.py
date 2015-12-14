#!/usr/bin/env python
"""Kinematic function skeleton code for Prelab 3.
Course: EE 106A, Fall 2015
Written by: Aaron Bestick, 9/10/14
Used by: EE106A, 9/11/15

This Python file is a code skeleton for Pre lab 3. You should fill in 
the body of the eight empty methods below so that they implement the kinematic 
functions described in the homework assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.

This code requires the NumPy and SciPy libraries. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import math
import numpy as np
import pylab as sp
from pylab import linalg

np.set_printoptions(precision=4,suppress=True)

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    a = omega[0]
    b = omega[1]
    c = omega[2]
    
    omega_hat = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    
    rot = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta), math.cos(theta)]])

    return rot

def norm_3vector(omega):
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    return math.sqrt(math.pow(omega[0],2) + math.pow(omega[1],2) + math.pow(omega[2],2))

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    i = np.eye(3)
    omega_norm = norm_3vector(omega)
    omega_hat = skew_3d(omega)

    rot = i + (omega_hat*(1/omega_norm)*math.sin(omega_norm*theta)) + (np.dot(omega_hat, omega_hat)*(1/(math.pow(omega_norm,2)))*(1-math.cos(omega_norm*theta)))

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    a = xi[0]
    b = xi[1]
    c = xi[2]

    xi_hat = np.array([[0,-c,a],[c,0,b],[0,0,0]])

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    a = xi[3]
    b = xi[4]
    c = xi[5]

    d = xi[0]
    e = xi[1]
    f = xi[2]
    
    xi_hat = np.array([[0,-c,b,d],[c,0,-a,e],[-b,a,0,f],[0,0,0,0]])

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    vx = xi[0]
    vy = xi[1]
    omega = xi[2]

    vxo = vx/omega
    vyo = vy/omega
    ot = omega*theta

    p_part1 = np.array([[1-math.cos(omega*theta), math.sin(omega*theta)],[-math.sin(omega*theta),1-math.cos(omega*theta)]])

    p_part2 = np.array([[0,-1],[1,0]])
    p_part3 = np.array([vxo,vyo])

    p_halfway = np.dot(p_part2, p_part3)
    p = np.dot(p_part1, p_halfway)
    p0 = p[0]
    p1 = p[1]
    
    g = np.array([[math.cos(ot), -math.sin(ot), p0],[math.sin(ot), math.cos(ot), p1],[0,0,1]])

    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    omega = xi[3:6]
    v = xi[0:3]
    vx = xi[0]
    vy = xi[1]
    vz = xi[2]
    wx = xi[3]
    wy = xi[4]
    wz = xi[5]

    rot = rotation_3d(omega, theta)

    vector1 = np.dot((np.eye(3)-rot), np.dot(skew_3d(omega),v))
    vector2 = np.dot(omega, np.dot(np.transpose(omega), v)) * theta
    vector = (1/math.pow(norm_3vector(omega),2)) * (vector1+vector2)
    vector2d = np.array([[vector[0],vector[1],vector[2]]])

    g = np.concatenate((rot, vector2d.T), axis=1)
    bottom = np.array([[0,0,0,1]])
    g = np.concatenate((g, bottom), axis=0)

    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    first = homog_3d(xi[:,0],theta[0])
    second = homog_3d(xi[:,1],theta[1])
    product = np.dot(first, second)
    
    n = xi.shape[1]
    x = 2
    while x < n:
        temp = homog_3d(xi[:,x],theta[x])
        product = np.dot(product, temp)
        x += 1

    g = product

    return g


def baxter_arm_FK(theta_joints):
	w1 = np.array([[-.0059],[.0113],[.9999]])
	q1 = np.array([[.0635],[.2598],[.1188]])
	w2 = np.array([[-.7077],[.7065],[-.0122]])
	q2 = np.array([[.1106],[.3116],[.3885]])
	w3 = np.array([[.7065],[.7077],[-.0038]])
	q3 = np.array([[.1827],[.3838],[.3881]])
	w4 = np.array([[-.7077],[.7065],[-.0122]])
	q4 = np.array([[.3682],[.5684],[.3181]])
	w5 = np.array([[.7065],[.7077],[-.0038]])
	q5 = np.array([[.4417],[.6420],[.3177]])
	w6 = np.array([[-.7077],[.7065],[-.0122]])
	q6 = np.array([[.6332],[.8337],[.3067]])
	w7 = np.array([[.7065],[.7077],[-.0038]])
	q7 = np.array([[.7152],[.9158],[.3063]])
	qhand = np.array([[.7957],[.9965],[.3058]])

	v1 = np.cross(-w1.T,q1.T)
	v1 = v1.T
	v2 = np.cross(-w2.T,q2.T)
	v2 = v2.T
	v3 = np.cross(-w3.T,q3.T)
	v3 = v3.T
	v4 = np.cross(-w4.T,q4.T)
	v4 = v4.T
	v5 = np.cross(-w5.T,q5.T)
	v5 = v5.T
	v6 = np.cross(-w6.T,q6.T)
	v6 = v6.T
	v7 = np.cross(-w7.T,q7.T)
	v7 = v7.T
	
	xi1 = np.concatenate((v1,w1),axis = 0)
	xi2 = np.concatenate((v2,w2),axis = 0)
	xi3 = np.concatenate((v3,w3),axis = 0)
	xi4 = np.concatenate((v4,w4),axis = 0)
	xi5 = np.concatenate((v5,w5),axis = 0)
	xi6 = np.concatenate((v6,w6),axis = 0)
	xi7 = np.concatenate((v7,w7),axis = 0)

	#print(xi1)
	#print(xi7)

	g0 = np.concatenate((np.eye(3),qhand),axis=1)
        g0 = np.concatenate((g0,np.array([[0,0,0,1]])),axis=0)
	#print("g0=" + str(g0))	

	xi = np.concatenate((xi1,xi2,xi3,xi4,xi5,xi6,xi7),axis=1)
        #print("xi=" + str(xi))

        prod = prod_exp(xi, theta_joints)
	
	return np.dot(prod, g0)

#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')

if __name__ == "__main__":
    print('Testing...')

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3.,  0.,  1.],
                            [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                            [ 0.9198, -0.3924,  1.2348],
                            [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    print('Done!')
