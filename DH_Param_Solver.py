from sympy import symbols, var
from sympy.matrices import Matrix
import numpy as np

# i | theta |  d | a | alpha
# ----------------------------
# 1 |	t1 	| 0  | 0 |	90
# 2 |	t2	| 0  | l2| 	0
# 3 |	t3	| l1 | l3| 	0
# ----------------------------

def GetRotMat(r, p, y):
	R_x = Matrix([
		[1, 0, 0, 0], [0, np.cos(r), -np.sin(r), 0], [0, np.sin(r), np.cos(r), 0], [0, 0, 0, 1]
		])

	R_y =Matrix([
		[np.cos(p), 0, np.sin(p), 0], [0, 1, 0, 0], [-np.sin(p), 0, np.cos(p), 0], [0, 0, 0, 1]
		])

	R_z = Matrix([
		[np.cos(y), -np.sin(y), 0, 0], [np.sin(y), np.cos(y), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
		])


	return R_x.multiply(R_y.multiply(R_z))

def GetTransform(d, a, alpha, i):
	s, c = var(f"s{i}, c{i}")
	T1 = Matrix([
				[c, -s, 0, 0],
				[s,  c, 0, 0],
				[0,  0, 1, 0],
				[0,  0, 0, 1]
				])
	T2 = Matrix([
				[1, 0, 0, 0],
				[0, 1, 0, 0],
				[0, 0, 1, d],
				[0, 0, 0, 1],
				])
	T3 = Matrix([
				[1, 0, 0, a],
				[0, 1, 0, 0],
				[0, 0, 1, 0],
				[0, 0, 0, 1],
				])
	T4 = Matrix([
				[1, 			0, 				0, 0],
				[0, np.cos(alpha), -np.sin(alpha), 0],
				[0, np.sin(alpha),  np.cos(alpha), 0],
				[0, 			0, 				0, 1],
				])

	T = T1.multiply(T2.multiply(T3.multiply(T4)))

	return T

l1, l2, l3 = var('l1, l2, l3')

T_r = GetRotMat(r = 0, p = 0, y = 0)

T_0_1 = GetTransform(d = 0, a = 0,  alpha = -np.pi/2, i = 1)
T_1_2 = GetTransform(d = 0, a = l2, alpha = 0, i = 2)
T_2_3 = GetTransform(d = l1, a = l3, alpha = 0, i = 3)

# T_0_3 = T_r.multiply(T_0_1.multiply(T_1_2.multiply(T_2_3)))
T_0_3 = T_0_1.multiply(T_1_2.multiply(T_2_3))



print(f"X: {T_0_3[0, -1]}")
print(f"Y: {T_0_3[1, -1]}")
print(f"Z: {T_0_3[2, -1]}")