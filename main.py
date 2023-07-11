import numpy as np
import pybullet as p
import pybullet_data
import time


from Robot import Robot


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
planeId = p.loadURDF("plane.urdf")



p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
p.setTimeStep(0.01) #THe lower this is, more accurate the simulation 
p.setRealTimeSimulation(0)  # we want to be faster than real time :

robot = Robot("Manipulator_3DoF.urdf", [0,0,0.5], [0,0,0])

for i in range(10000):
	p.stepSimulation()
	time.sleep(1./240.)