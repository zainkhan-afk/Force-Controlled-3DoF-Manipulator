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
robot.MoveTo(x, y, z)
for i in range(10000):
	state = robot.GetCurrentState()
	J_robotjointbase, J_robotframe = robot.GetJabobian(state.GetPosition())

	if np.linalg.det(J_robotjointbase) != 0:
		J_robotjointbase_inv = np.linalg.inv(J_robotjointbase)

		torque = state.GetTorque()[:, np.newaxis]
		force = J_robotjointbase@torque
		torque_calculated = J_robotjointbase_inv@force

		torque_str = f"Torque - ({round(torque[0, 0], 2)}, {round(torque[1, 0], 2)}, {round(torque[2, 0], 2)})"
		force_str = f"Force - ({round(force[0, 0], 2)}, {round(force[1, 0], 2)}, {round(force[2, 0], 2)})"
		torque_calculated_str = f"Force - ({round(torque_calculated[0, 0], 2)}, {round(torque_calculated[1, 0], 2)}, {round(torque_calculated[2, 0], 2)})"

		print(f"{torque_str}, {force_str}, {torque_calculated_str}")


	p.stepSimulation()
	time.sleep(1./240.)

	x = x_base + 0.5*np.sin(pos)
	# y = y_base + 0.5*np.sin(pos)
	# z = z_base + 0.5*np.sin(pos)
	pos += 0.01