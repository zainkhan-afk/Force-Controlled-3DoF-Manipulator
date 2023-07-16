import numpy as np
from utils import *

class Kinematics:
	def __init__(self):
		self.l1 = 0.3
		self.l2 = 1
		self.l3 = 1


	def FK(self, theta1, theta2, theta3):
		# X: c1*(c2*c3*l3 + c2*l2 - 1.0*l3*s2*s3) - 1.0*l1*s1
		# Y: 1.0*c1*l1 + s1*(c2*c3*l3 + c2*l2 - 1.0*l3*s2*s3)
		# Z: -1.0*c2*l3*s3 - 1.0*c3*l3*s2 - 1.0*l2*s2

		q = [theta1, theta2, theta3]
		x = 0
		y = 0
		z = 0

		# -90 in DH and -90 in Robot
		x = - self.l1*trig_solve('s', [q[0]]) + trig_solve('c', [q[0]])*(self.l3*trig_solve('cc', q[1:]) + self.l2*trig_solve('c', [q[1]]) - self.l3*trig_solve('ss', q[1:]))
		y =   self.l1*trig_solve('c', [q[0]]) + trig_solve('s', [q[0]])*(self.l3*trig_solve('cc', q[1:]) + self.l2*trig_solve('c', [q[1]]) - self.l3*trig_solve('ss', q[1:]))
		z = - self.l3*trig_solve('cs', q[1:]) - self.l3*trig_solve('sc', q[1:]) - self.l2*trig_solve('s', [q[1]]) 

		return x, y, z


	def IK(self, x, y, z):
		R = np.sqrt(z**2 + y**2)

		beta = np.arccos(y/R)
		alpha  = np.arccos(self.l1/R)

		theta1 = alpha - beta

		R_x = get_rot_mat(-theta1, 0, 0)
		R_yz = get_rot_mat(0, -np.pi/2, np.pi)

		p = np.array([[x, y, z]]).T
		p = R_yz@(R_x@p)

		x_ = p[0, 0]
		y_ = p[1, 0] + self.l1
		z_ = p[2, 0]


		temp = (x_**2 + z_**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta3 =   np.arccos(temp)
		theta2 =   (np.arctan2(z_, x_) - np.arctan2(self.l3*np.sin(theta3),(self.l2 + self.l3*np.cos(theta3))))

		return theta1, theta2, theta3