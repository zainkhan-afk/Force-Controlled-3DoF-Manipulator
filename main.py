import numpy as np
import pybullet as p
import pybullet_data
import time


from Robot import Robot
from utils import *


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
planeId = p.loadURDF("plane.urdf")



p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
p.setTimeStep(0.01) #THe lower this is, more accurate the simulation 
p.setRealTimeSimulation(0)  # we want to be faster than real time :

robot = Robot("Manipulator_3DoF.urdf", [0, 0, 0.1], [0, 0, 0])


x_base = 1.5
y_base = robot.kine_model.l1
# y_base = 0
z_base = -1

x = x_base
y = y_base
z = z_base

pos = 0

R = get_rot_mat(x = 0, y = -np.pi/2, z = 0)
for i in range(10000):
	robot.MoveTo(x, y, z)
	# desired_pos = np.array([
	# 						Deg2Rad(0*np.sin(pos)),
	# 						Deg2Rad(0*np.sin(pos)),
	# 						Deg2Rad(45*np.sin(pos))
	# 						])
	# robot.MoveJoints(desired_pos)
	# x_fk, y_fk, z_fk = robot.kine_model.FK(desired_pos[0], desired_pos[1], desired_pos[2])
	# pos_world = np.array([[x_fk, y_fk, z_fk]])
	# pos_world = R.T@pos_world.T
	# x_fk, y_fk, z_fk = pos_world[0, 0], pos_world[1, 0], pos_world[2, 0]

	# print(f"Position: {(round(x_fk, 2))}, {(round(y_fk, 2))}, {(round(z_fk, 2))} - Angle: {desired_pos*180/np.pi}")

	p.stepSimulation()
	time.sleep(1./240.)

	# x = x_base + 0.5*np.sin(pos)
	# y = y_base + 0.5*np.sin(pos)
	# z = z_base + 0.5*np.sin(pos)
	pos += 0.001