import numpy as np
from utils import *

class Kinematics:
	def __init__(self):
		self.l1 = 0.5
		self.l2 = 1.5
		self.l3 = 1.5

		self.l1 = 0.077476
		self.l2 = 0.2115
		self.l3 = 0.2


		self.l1 = 0.062
		self.l2 = 0.209
		self.l3 = 0.18

	def FK(self, theta1, theta2, theta3):
		c1c2c3 = np.cos(theta1)*np.cos(theta2)*np.cos(theta3)
		c1s2s3 = np.cos(theta1)*np.sin(theta2)*np.sin(theta3)
		s1c2c3 = np.sin(theta1)*np.cos(theta2)*np.cos(theta3)
		s1s2s3 = np.sin(theta1)*np.sin(theta2)*np.sin(theta3)

		c1c2 = np.cos(theta1)*np.cos(theta2)
		s1c2 = np.sin(theta1)*np.cos(theta2)
		s2c3 = np.sin(theta2)*np.cos(theta3)
		c2s3 = np.cos(theta2)*np.sin(theta3)



		x = self.l3*(c1c2c3 - c1s2s3) + self.l2*c1c2 + self.l1*np.sin(theta1)
		y = self.l3*(s1c2c3 - s1s2s3) + self.l2*s1c2 - self.l1*np.cos(theta1)
		z = self.l3*(s2c3 - c2s3) + self.l2*np.sin(theta2)

		R = get_rot_mat(x = 0, y = np.pi/2, z = np.pi)
		p = np.array([[x, y, z]])
		p = R@p.T

		x = p[0,0]
		y = p[1,0]
		z = p[2,0]

		return x, y, z

	def IK(self, x, y, z):
		R = np.sqrt(z**2 + y**2)

		alpha = np.arccos(abs(y)/R)
		beta  = np.arccos(self.l1/R)

		if y>=0:
			theta1 = alpha - beta
		else:
			theta1 = np.pi - alpha - beta

		x_ = x
		z_ = -np.sqrt(z**2+y**2-self.l1**2)


		temp = (x_**2 + z_**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta3 =   np.arccos(temp)
		theta2 =   np.pi/2 + (np.arctan2(z_, x_) - np.arctan2(self.l3*np.sin(theta3),(self.l2 + self.l3*np.cos(theta3))))

		return theta1, theta2, theta3