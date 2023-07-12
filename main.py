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

p_body = np.array([[x, y, z]])
R = get_rot_mat(0, -np.pi/2, 0)

pos = 0
for i in range(10000):
	p_leg = R@p_body.T
	
	robot.MoveTo(x, y, z)

	p.stepSimulation()
	time.sleep(1./240.)

	# x = x_base + 0.5*np.sin(pos)
	# y = y_base + 0.5*np.sin(pos)
	# z = z_base + 0.5*np.sin(pos)
	# pos += 0.01