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


x_base = 0
y_base = robot.kine_model.l1
# y_base = 0
z_base = -1

x = x_base
y = y_base
z = z_base

pos = 0

R = get_rot_mat(x = 0, y = -np.pi/2, z = 0)
for i in range(10000):
	# robot.MoveTo(x, y, z)

	state = robot.GetCurrentState()
	J_robotjointbase, J_robotframe = robot.GetJabobian(state.GetPosition())

	q_dot = state.GetTorque()[:, np.newaxis]

	v_dot = J_robotjointbase@q_dot
	v_dot = robot.R_robotframe_robotjointbase.T@v_dot

	print(q_dot.T)

	p.stepSimulation()
	time.sleep(1./240.)

	x = x_base + 0.5*np.sin(pos)
	# y = y_base + 0.5*np.sin(pos)
	# z = z_base + 0.5*np.sin(pos)
	pos += 0.01