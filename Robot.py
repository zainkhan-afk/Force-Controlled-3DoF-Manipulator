import pybullet as p
import numpy as np

from Kinematics import Kinematics
from utils import *
from State import State

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

		self.R_robotframe_robotjointbase = get_rot_mat(0, -np.pi/2, 0)


	def MoveTo(self, x, y, z):
		pos = np.array([[x, y, z]]).T
		# pos = self.R_robotframe_robotjointbase@pos

		x = pos[0, 0]
		y = pos[1, 0]
		z = pos[2, 0]

		t1, t2, t3 = self.kine_model.IK(x, y, z)
		x_, y_, z_ = self.kine_model.FK(t1, t2, t3)

		pos_ = np.array([[x_, y_, z_]]).T
		pos_ = self.R_robotframe_robotjointbase.T@pos_

		x_ = pos_[0, 0]
		y_ = pos_[1, 0]
		z_ = pos_[2, 0]


		desired = [t1, t2, t3]
		desired = np.array(desired)

		self.MoveJoints(desired)
		
		pos_str = f"{round(x, 2), round(y, 2), round(z, 2)}"
		pos_FK_str = f"{round(x_, 2), round(y_, 2), round(z_, 2)}"
		desired_pos_str = f"{round(t1*180/np.pi, 2), round(t2*180/np.pi, 2), round(t3*180/np.pi, 2)}"

		# print(f"Requested Pos: {pos_str} - FK Pos: {pos_FK_str} - Desired Joint Pos {desired_pos_str}")
		

	def MoveJoints(self, desired):
		current_state = self.GetCurrentState()

		error = desired - current_state.GetPosition()
		
		# diff_error = self.joint_position_prev_errors - error
		diff_error = error - self.joint_position_prev_errors 
		
		self.joint_position_errors_sum += error

		val = current_state.GetPosition() + self.P*error + self.D*diff_error + self.I*self.joint_position_errors_sum

		# desired_str = f"{round(desired[0], 2), round(desired[1], 2), round(desired[1], 2)}"
		# current_str = f"{round(current[0], 2), round(current[1], 2), round(current[1], 2)}"
		# val_str = f"{round(val[0], 2), round(val[1], 2), round(val[1], 2)}"
		# error_str = f"{round(error[0], 2), round(error[1], 2), round(error[1], 2)}"


		# print(f"desired: {desired_str} - current: {current_str} - Control: {val_str} - Error: {error_str}")

		p.setJointMotorControlArray(self.robot, self.joint_indices, p.POSITION_CONTROL, targetPositions = val)

		self.joint_position_prev_errors = error.copy()


	def GetCurrentState(self):
		joint_states = p.getJointStates(self.robot, self.joint_indices)

		current = []
		for val in joint_states:
			current += [val[0], val[1], val[-1]]

		current = np.array(current)

		current_state = State(current)

		return current_state

	def GetJabobian(self, q):
		# x = - l1*trig_solve('s', [q[0]]) + trig_solve('c', [q[0]])*(l3*trig_solve('cc', q[1:]) + l2*trig_solve('c', [q[1]]) - l3*trig_solve('ss', q[1:]))
		# y =   l1*trig_solve('c', [q[0]]) + trig_solve('s', [q[0]])*(l3*trig_solve('cc', q[1:]) + l2*trig_solve('c', [q[1]]) - l3*trig_solve('ss', q[1:]))
		# z = - l3*trig_solve('cs', q[1:]) - l3*trig_solve('sc', q[1:]) - l2*trig_solve('s', [q[1]]) 

		# t1, t2, t3 = q
		l1, l2, l3 = self.kine_model.l1, self.kine_model.l2, self.kine_model.l3

		dx_dt1 = - l1*trig_solve('c', [q[0]]) - trig_solve('s', [q[0]])*(l3*trig_solve('cc', q[1:]) + l2*trig_solve('c', [q[1]]) - l3*trig_solve('ss', q[1:]))
		dx_dt2 = trig_solve('c', [q[0]])*(-l3*trig_solve('sc', q[1:]) - l2*trig_solve('s', [q[1]]) - l3*trig_solve('cs', q[1:]))
		dx_dt3 = trig_solve('c', [q[0]])*(-l3*trig_solve('cs', q[1:]) - l3*trig_solve('sc', q[1:]))

		dy_dt1 = - l1*trig_solve('s', [q[0]]) + trig_solve('c', [q[0]])*(l3*trig_solve('cc', q[1:]) + l2*trig_solve('c', [q[1]]) - l3*trig_solve('ss', q[1:]))
		dy_dt2 = trig_solve('s', [q[0]])*(-l3*trig_solve('sc', q[1:]) - l2*trig_solve('s', [q[1]]) - l3*trig_solve('cs', q[1:]))
		dy_dt3 = trig_solve('s', [q[0]])*(-l3*trig_solve('cs', q[1:]) - l3*trig_solve('sc', q[1:]))

		dz_dt1 = 0
		dz_dt2 = l3*trig_solve('ss', q[1:]) - l3*trig_solve('cc', q[1:]) - l2*trig_solve('c', [q[1]])
		dz_dt3 = - l3*trig_solve('cc', q[1:]) + l3*trig_solve('ss', q[1:])

		J_robotjointbase = np.array([
						[dx_dt1, dx_dt2, dx_dt3],
						[dy_dt1, dy_dt2, dy_dt3],
						[dz_dt1, dz_dt2, dz_dt3]
					])

		J_robotframe = self.R_robotframe_robotjointbase.T@J_robotjointbase

		return J_robotjointbase, J_robotframe