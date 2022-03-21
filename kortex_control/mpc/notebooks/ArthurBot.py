import numpy as np
import math
import time
import random

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ArthurBot:

	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		self.Rdesc=[
			[3.1416, 2.7629E-18, -4.9305E-36, 0, 0, 0.15643], # From robot base to joint1
			[1.5708, 2.1343E-17, -1.1102E-16, 0, 0.005375, -0.12838],
			[-1.5708, 1.2326E-32, -2.9122E-16, 0, -0.21038, -0.006375],
			[1.5708, -6.6954E-17, -1.6653E-16, 0, 0.006375, -0.21038],
			[-1.5708, 2.2204E-16, -6.373E-17, 0, -0.20843, -0.006375],
			[1.5708, 9.2076E-28, -8.2157E-15, 0, 0.00017505, -0.10593],
			[-1.5708, -5.5511E-17, 9.6396E-17, 0, -0.10593, -0.00017505],
			[3.14159265358979, 1.09937075168372E-32, 0, 0, 0, -0.0615250000000001] # From joint7 to end-effector center
			]
            
		#Define the axis of rotation for each joint
		self.axis=[
            		[0, 0, 1],
            		[0, 0, 1],
            		[0, 0, 1],
            		[0, 0, 1],
                    [0, 0, 1],
					[0, 0, 1],
					[0, 0, 1],
                    [0, 0, 1]
                    ]

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]]
		
		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])
		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])

		# initialize Jacobian matrix
		self.J=np.zeros((6,7))
		
		self.q=[0.,0.,0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.,0.,0.,0.])


		# NEW #############################
		# joint limits		
		self.qmin=[float('-inf'), math.radians(-128.9), float('-inf'), math.radians(-147.8), float('-inf'), math.radians(-120.3), float('-inf')] # NOTE-does not include grippers
		self.qmax=[float('inf'), math.radians(128.9), float('inf'), math.radians(147.8), float('inf'), math.radians(120.3), float('inf')] # NOTE-does not include grippers


	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''
		self.q[0:-1]=ang
		
		#Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!
		for i in range(len(self.Rdesc)):
			if self.axis[i] == [0,0,1]:
				self.Tjoint[i]=[[math.cos(self.q[i]),-math.sin(self.q[i]),0,0],[math.sin(self.q[i]),math.cos(self.q[i]),0,0],[0,0,1,0],[0,0,0,1]]
			elif self.axis[i] == [-1,0,0]:
				self.Tjoint[i]=[[1,0,0,0],[0,math.cos(self.q[i]), math.sin(self.q[i]),0],[0,-math.sin(self.q[i]),math.cos(self.q[i]),0],[0,0,0,1]]
			elif self.axis[i] == [0,1,0]:
				self.Tjoint[i]=[[math.cos(self.q[i]),0,math.sin(self.q[i]),0],[0,1,0,0],[-math.sin(self.q[i]),0,math.cos(self.q[i]),0],[0,0,0,1]]
			else:
				raise ValueError('Axis rotation is not defined')

			if i == 0:
				self.Tcurr[i]=np.matmul(self.Tlink[i],self.Tjoint[i])
			else:
				self.Tcurr[i]=np.matmul(np.matmul(self.Tcurr[i-1],self.Tlink[i]),self.Tjoint[i])
		
		# Compute Jacobian matrix		
		for i in range(len(self.Tcurr)-1):
			p=self.Tcurr[-1][0:3,3]-self.Tcurr[i][0:3,3]
			# tells which axis the joint is rotating about
			ax_of_rotation = np.nonzero(self.axis[i])[0][0]	
			a=self.Tcurr[i][0:3,ax_of_rotation]*np.sign(self.axis[i][ax_of_rotation])
			self.J[0:3,i]=np.cross(a,p)
			self.J[3:7,i]=a

		return self.Tcurr, self.J




	def IterInvKin(self,ang,TGoal,x_eps=1e-3, r_eps=1e-3):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	
		step_size_r = 0.5
		step_size_p = 0.02
		
		self.ForwardKin(ang)
		
		Err=[0.,0.,0.,0.,0.,0.] # error in position and orientation, initialized to 0
		for s in range(1000):
			#Compute rotation error
			rErrR=np.matmul(TGoal[0:3,0:3],np.transpose(self.Tcurr[-1][0:3,0:3]))
			rErrAxis,rErrAng=rt.R2axisang(rErrR)
			if rErrAng>step_size_r:
				rErrAng=step_size_r
			if rErrAng<-step_size_r:
				rErrAng=-step_size_r
			rErr= [rErrAxis[0]*rErrAng,rErrAxis[1]*rErrAng,rErrAxis[2]*rErrAng]
			Err[3:6]=rErr			

			#Compute position error
			xErr=TGoal[0:3,3]-self.Tcurr[-1][0:3,3]
			if np.linalg.norm(xErr)>step_size_p:
				xErr= xErr*step_size_p/np.linalg.norm(xErr)
			Err[0:3]=xErr
			
			# if norm of the error is below thresholds, then exit 
			if np.linalg.norm(Err[0:3]) <= x_eps and np.linalg.norm(Err[3:6]) <= r_eps:
				print("found IK solution with err:",np.linalg.norm(Err[0:3]),np.linalg.norm(Err[3:6]))
				break

			#Update joint angles
			C=np.diag([1e-6,1e-6,1e-6,1e-6,1e-6,1e-4])
			self.q[0:-1]=self.q[0:-1]+np.matmul(np.matmul(np.transpose(self.J[0:6,:]),np.linalg.inv(C+np.matmul(self.J[0:6,:],np.transpose(self.J[0:6,:])))),Err)
		

			#Recompute forward kinematics for new angles
			self.ForwardKin(self.q[0:-1])

		
		return self.q[0:-1], Err


	def PlotSkeleton(self,ang):
		#Compute forward kinematics for ang
		self.ForwardKin(ang)

		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')

		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i == 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		#Format axes and display
		ax.axis('equal')
		ax.set(xlim=(-1., 1.), ylim=(-1., 1.), zlim=(0,1.))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()

	


