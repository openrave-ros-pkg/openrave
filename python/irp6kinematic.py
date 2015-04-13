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


def solveIKPost(env,goalRQ,goalT):
	
	goalRM = quaternion_to_matrix(goalRQ);
	
	desiredPosition=""
	for i in goalRM:
		desiredPosition = desiredPosition + " " + str(i)
	for i in goalT:
		desiredPosition = desiredPosition + " " + str(i)
	arguments = " " + desiredPosition

	#Call plugin 
	prob = RaveCreateModule(env,"irp6kinematic")
	env.AddModule(prob,args='')
	strResult = prob.SendCommand('solveIKPost' + arguments) # whole list as a string
	#print strResult
	
	#convert string to list of numbers
	sresult = strResult.split() # list of strings
	result = [] # list of numbers
	for i in sresult:
		result.append(float(i))
	return result
	
def solveIKTrack(env,goalRQ,goalT):
	
	goalRM = quaternion_to_matrix(goalRQ);
	
	desiredPosition=""
	for i in goalRM:
		desiredPosition = desiredPosition + " " + str(i)
	for i in goalT:
		desiredPosition = desiredPosition + " " + str(i)
	arguments = " " + desiredPosition

	#Call plugin 
	prob = RaveCreateModule(env,"irp6kinematic")
	env.AddModule(prob,args='')
	strResult = prob.SendCommand('solveIKTrack' + arguments) # whole list as a string
	#print strResult
	
	#convert string to list of numbers
	sresult = strResult.split() # list of strings
	result = [] # list of numbers
	for i in sresult:
		result.append(float(i))
	return result
	
