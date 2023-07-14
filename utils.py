import numpy as np

def get_rot_mat( x, y, z):
	R_x = np.array([
		[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]
		])

	R_y = np.array([
		[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]
		])

	R_z = np.array([
		[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]
		])


	return R_x@R_y@R_z


def get_inverse_transformation(T):
	R = T[0:3, 0:3]
	trans = T[:-1, -1][:, np.newaxis]

	new_R = np.linalg.inv(R)
	new_trans = -new_R@trans

	T = np.append(new_R, new_trans, axis = 1)
	T = np.append(T, np.array([[0, 0, 0, 1]]), axis = 0)

	return T

def trig_solve(chain, angle_list):
	chain = chain.lower()
	i = 0
	product = 1
	for trig_func in chain:
		if trig_func == "s":
			product *= np.sin(angle_list[i])
		if trig_func == "c":
			product *= np.cos(angle_list[i])

		i += 1

	return product


def Rad2Deg(rad):
	return rad/np.pi*180

def Deg2Rad(deg):
	return deg/180*np.pi