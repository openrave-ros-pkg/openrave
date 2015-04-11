#!/usr/bin/env python

"""Solves IK problem for irp6

	it uses plugin irp6iksolver

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Przemyslaw Krajewski'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    _EPS = numpy.finfo(float).eps * 4.0
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0],
            q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0],
            q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]])
def quaternion_to_matrix(q):
	X=q[0];
	Y=q[1];
	Z=q[2];
	W=q[3];
	return numpy.array([
	1.0-2*Y*Y-2*Z*Z,	2*X*Y-2*Z*W,	2*X*Z+2*Y*W,
	2*X*Y+2*Z*W,		1-2*X*X-2*Z*Z,	2*Y*Z-2*X*W,
	2*X*Z-2*Y*W,		2*Y*Z+2*X*W,	1-2*X*X-2*Y*Y
	])

def solveFKPost(env,robot):
	currentJoint1 = robot.GetJoint('Irp6pmJoint1').GetValue(0)
	currentJoint2 = robot.GetJoint('Irp6pmJoint2').GetValue(0)
	currentJoint3 = robot.GetJoint('Irp6pmJoint3').GetValue(0)
	currentJoint4 = robot.GetJoint('Irp6pmJoint4').GetValue(0)
	currentJoint5 = robot.GetJoint('Irp6pmJoint5').GetValue(0)
	currentJoint6 = robot.GetJoint('Irp6pmJoint6').GetValue(0)
	
	#Convert data to stream/string
	currentJoints =  str(currentJoint1) + "  " + str(currentJoint2) + "  " + str(currentJoint3) + "  " + str(currentJoint4) + "  " + str(currentJoint5) + "  " + str(currentJoint6)
	
	arguments = " " + currentJoints
	
	prob = RaveCreateModule(env,"irp6kinematic")
	env.AddModule(prob,args='')
	strResult = prob.SendCommand('fkSolvePost' + arguments) # whole list as a string


def solveIKPost(env,robot,goalRQ,goalT):
	#Getting current Joints of robot
	currentJoint1 = robot.GetJoint('Irp6pmJoint1').GetValue(0)
	currentJoint2 = robot.GetJoint('Irp6pmJoint2').GetValue(0)
	currentJoint3 = robot.GetJoint('Irp6pmJoint3').GetValue(0)
	currentJoint4 = robot.GetJoint('Irp6pmJoint4').GetValue(0)
	currentJoint5 = robot.GetJoint('Irp6pmJoint5').GetValue(0)
	currentJoint6 = robot.GetJoint('Irp6pmJoint6').GetValue(0)
	
	#Convert data to stream/string
	currentJoints =  str(currentJoint1) + "  " + str(currentJoint2) + "  " + str(currentJoint3) + "  " + str(currentJoint4) + "  " + str(currentJoint5) + "  " + str(currentJoint6)
	
	goalRM = quaternion_to_matrix(goalRQ);
	
	desiredPosition=""
	for i in goalRM:
		desiredPosition = desiredPosition + " " + str(i)
	for i in goalT:
		desiredPosition = desiredPosition + " " + str(i)
	arguments = " " + currentJoints + " " + desiredPosition

	#Call plugin 
	prob = RaveCreateModule(env,"irp6kinematic")
	env.AddModule(prob,args='')
	strResult = prob.SendCommand('ikSolvePost' + arguments) # whole list as a string
	#print strResult
	
	#convert string to list of numbers
	sresult = strResult.split() # list of strings
	result = [] # list of numbers
	for i in sresult:
		result.append(float(i))
	return result
	
