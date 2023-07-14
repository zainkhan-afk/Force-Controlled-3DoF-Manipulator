import pybullet as p
import numpy as np

from Kinematics import Kinematics
from utils import *

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

		self.P = 1
		self.I = 0
		self.D = 0

		self.joint_position_prev_errors = np.zeros(3)
		self.joint_position_errors_sum = np.zeros(3)

		self.kine_model = Kinematics()

		self.joint_indices = self.joint_dict.keys()

		self.R_world_robot = get_rot_mat(0, -np.pi/2, 0)


	def MoveTo(self, x, y, z):
		pos = np.array([[x, y, z]]).T
		pos = self.R_world_robot@pos

		x = pos[0, 0]
		y = pos[1, 0]
		z = pos[2, 0]

		t1, t2, t3 = self.kine_model.IK(x, y, z)
		x_, y_, z_ = self.kine_model.FK(t1, t2, t3)


		desired = [t1, t2, t3]
		desired = np.array(desired)

		self.MoveJoints(desired)
		
		pos_str = f"{round(x, 2), round(y, 2), round(z, 2)}"
		pos_FK_str = f"{round(x_, 2), round(y_, 2), round(z_, 2)}"

		print(f"Requested Pos: {pos_str} - FK Pos: {pos_FK_str} - R {np.sqrt(x**2 + y**2 + z**2)}")
		

	def MoveJoints(self, desired):
		current = self.GetCurrentJointPosition()

		error = desired - current
		
		# diff_error = self.joint_position_prev_errors - error
		diff_error = error - self.joint_position_prev_errors 
		
		self.joint_position_errors_sum += error

		val = current + self.P*error + self.D*diff_error + self.I*self.joint_position_errors_sum

		# desired_str = f"{round(desired[0], 2), round(desired[1], 2), round(desired[1], 2)}"
		# current_str = f"{round(current[0], 2), round(current[1], 2), round(current[1], 2)}"
		# val_str = f"{round(val[0], 2), round(val[1], 2), round(val[1], 2)}"
		# error_str = f"{round(error[0], 2), round(error[1], 2), round(error[1], 2)}"


		# print(f"desired: {desired_str} - current: {current_str} - Control: {val_str} - Error: {error_str}")

		p.setJointMotorControlArray(self.robot, self.joint_indices, p.POSITION_CONTROL, targetPositions = val)

		self.joint_position_prev_errors = error.copy()


	def GetCurrentJointPosition(self):
		joint_states = p.getJointStates(self.robot, self.joint_indices)

		current = []
		for val in joint_states:
			current.append(val[0])

		current = np.array(current)

		return current