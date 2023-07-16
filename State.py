class State:
	def __init__(self, q):
		self.position = q[[0, 3, 6]]
		self.velocity = q[[1, 4, 7]]
		self.torque   = q[[2, 5, 8]]

	def SetState(self, q):
		self.position = q[[0, 3, 6]]
		self.velocity = q[[1, 4, 7]]
		self.torque   = q[[2, 5, 8]]

	def GetPosition(self):
		return self.position

	def GetVelocity(self):
		return self.velocity

	def GetTorque(self):
		return self.torque