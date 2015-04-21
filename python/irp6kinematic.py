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

class Irp6Kinematic:

	def __init__(self,env,robot):
		self.enviroment = env
		self.robot = robot
		
		self.prob = RaveCreateModule(self.enviroment,"irp6kinematic")
		self.enviroment.AddModule(self.prob,args='')
		
	def quaternion_to_matrix(self,q):
		X=q[0];
		Y=q[1];
		Z=q[2];
		W=q[3];
		return numpy.array([
		1.0-2*Y*Y-2*Z*Z,	2*X*Y-2*Z*W,	2*X*Z+2*Y*W,
		2*X*Y+2*Z*W,		1-2*X*X-2*Z*Z,	2*Y*Z-2*X*W,
		2*X*Z-2*Y*W,		2*Y*Z+2*X*W,	1-2*X*X-2*Y*Y
		])

	def solveFKPost(self):
		self.robot.SetActiveManipulator('postument');
		
		currentJoint1 = self.robot.GetJoint('Irp6pmJoint1').GetValue(0)
		currentJoint2 = self.robot.GetJoint('Irp6pmJoint2').GetValue(0)
		currentJoint3 = self.robot.GetJoint('Irp6pmJoint3').GetValue(0)
		currentJoint4 = self.robot.GetJoint('Irp6pmJoint4').GetValue(0)
		currentJoint5 = self.robot.GetJoint('Irp6pmJoint5').GetValue(0)
		currentJoint6 = self.robot.GetJoint('Irp6pmJoint6').GetValue(0)
		
		#Convert data to stream/string
		currentJoints =  str(currentJoint1) + "  " + str(currentJoint2) + "  " + str(currentJoint3) + "  " + str(currentJoint4) + "  " + str(currentJoint5) + "  " + str(currentJoint6)
		
		arguments = " " + currentJoints
		
		#Call plugin
		strResult = self.prob.SendCommand('solveFKPost' + arguments) # whole list as a string
		
		#convert string to list of numbers
		sresult = strResult.split() # list of strings
		result = [] # list of numbers
		for i in sresult:
			result.append(float(i))
		return result
		
	def solveFKTrack(self):
		self.robot.SetActiveManipulator('track');
		
		currentJoint1 = self.robot.GetJoint('Irp6otmJoint1').GetValue(0)
		currentJoint2 = self.robot.GetJoint('Irp6otmJoint2').GetValue(0)
		currentJoint3 = self.robot.GetJoint('Irp6otmJoint3').GetValue(0)
		currentJoint4 = self.robot.GetJoint('Irp6otmJoint4').GetValue(0)
		currentJoint5 = self.robot.GetJoint('Irp6otmJoint5').GetValue(0)
		currentJoint6 = self.robot.GetJoint('Irp6otmJoint6').GetValue(0)
		currentJoint7 = self.robot.GetJoint('Irp6otmJoint7').GetValue(0)
		
		#Convert data to stream/string
		currentJoints =  str(currentJoint1) + "  " + str(currentJoint2) + "  " + str(currentJoint3) + "  " + str(currentJoint4) + "  " + str(currentJoint5) + "  " + str(currentJoint6) + "  " + str(currentJoint7)
		arguments = " " + currentJoints
		
		#Call plugin
		strResult = self.prob.SendCommand('solveFKTrack' + arguments) # whole list as a string
		
		#convert string to list of numbers
		sresult = strResult.split() # list of strings
		result = [] # list of numbers
		for i in sresult:
			result.append(float(i))
		return result


	def solveIKPost(self,goalRQ,goalT):
		
		goalRM = self.quaternion_to_matrix(goalRQ);
		
		desiredPosition=""
		for i in goalRM:
			desiredPosition = desiredPosition + " " + str(i)
		for i in goalT:
			desiredPosition = desiredPosition + " " + str(i)
		arguments = " " + desiredPosition

		#Call plugin 
		strResult = self.prob.SendCommand('solveIKPost' + arguments) # whole list as a string
		#print strResult
		
		#convert string to list of numbers
		sresult = strResult.split() # list of strings
		result = [] # list of numbers
		for i in sresult:
			result.append(float(i))
		return result
		
	def solveIKTrack(self,goalRQ,goalT):
		
		goalRM = self.quaternion_to_matrix(goalRQ);
		
		desiredPosition=""
		for i in goalRM:
			desiredPosition = desiredPosition + " " + str(i)
		for i in goalT:
			desiredPosition = desiredPosition + " " + str(i)
		arguments = " " + desiredPosition

		#Call plugin 
		strResult = self.prob.SendCommand('solveIKTrack' + arguments) # whole list as a string
		#print strResult
		
		#convert string to list of numbers
		sresult = strResult.split() # list of strings
		result = [] # list of numbers
		for i in sresult:
			result.append(float(i))
		return result

	def solveRelativeIKPost(self,rotation,translation):
		self.robot.SetActiveManipulator('postument');
		
		currentJoint1 = self.robot.GetJoint('Irp6pmJoint1').GetValue(0)
		currentJoint2 = self.robot.GetJoint('Irp6pmJoint2').GetValue(0)
		currentJoint3 = self.robot.GetJoint('Irp6pmJoint3').GetValue(0)
		currentJoint4 = self.robot.GetJoint('Irp6pmJoint4').GetValue(0)
		currentJoint5 = self.robot.GetJoint('Irp6pmJoint5').GetValue(0)
		currentJoint6 = self.robot.GetJoint('Irp6pmJoint6').GetValue(0)
		#Convert data to stream/string
		currentJoints =  str(currentJoint1) + "  " + str(currentJoint2) + "  " + str(currentJoint3) + "  " + str(currentJoint4) + "  " + str(currentJoint5) + "  " + str(currentJoint6)
		
		arguments = " " + currentJoints
		
		desiredPosition=""
		for i in translation:
			desiredPosition = desiredPosition + " " + str(i)
		for i in rotation:
			desiredPosition = desiredPosition + " " + str(i)
		arguments = arguments + " " + desiredPosition

		#Call plugin 
		strResult = self.prob.SendCommand('solveRelativeIKPost' + arguments) # whole list as a string
		#print strResult
		
		#convert string to list of numbers
		sresult = strResult.split() # list of strings
		result = [] # list of numbers
		for i in sresult:
			result.append(float(i))
		return result		
