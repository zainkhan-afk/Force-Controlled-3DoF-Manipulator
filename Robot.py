import pybullet as p
import numpy as np

class Robot:
	def __init__(self, urdf_path, start_pos, start_orientation):
		self.start_pos = start_pos
		self.start_orientation = p.getQuaternionFromEuler(start_orientation)

		self.robot = p.loadURDF(urdf_path, self.start_pos, self.start_orientation)
		self.joint_dict = {}

		num_joints = p.getNumJoints(self.robot)
		for i in range(num_joints):
			joint_name = p.getJointInfo(self.robot, i)[1].decode("UTF-8")
			
			if 'toe' not in joint_name:
				self.joint_dict[i] = joint_name

		print(self.joint_dict)

		self.joint_indices = self.joint_dict.keys()