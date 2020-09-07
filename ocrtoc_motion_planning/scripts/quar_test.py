import numpy as np
from scipy.spatial.transform import Rotation as R

def quarternion_to_matrix(x, y, z, w):
    ori_matrix = np.zeros((3,3))

    ori_matrix[0,0] = 1 - 2*y*y - 2*z*z
    ori_matrix[0,1] = 2*x*y - 2*z*w
    ori_matrix[0,2] = 2*x*z + 2*y*w
    ori_matrix[1,0] = 2*x*y + 2*z*w
    ori_matrix[1,1] = 1-2*x*x - 2*z*z
    ori_matrix[1,2] = 2*y*z - 2*x*w
    ori_matrix[2,0] = 2*x*z - 2*y*w
    ori_matrix[2,1] = 2*y*z + 2*x*w
    ori_matrix[2,2] = 1-2*x*x - 2*y*y
    return ori_matrix

def matrix_to_quarternion(ori_matrix):
    w = np.sqrt(1+ori_matrix[0,0]+ori_matrix[1,1]+ori_matrix[2,2])/2
    x = (ori_matrix[2,1] - ori_matrix[1,2]) / (4*w)
    y = (ori_matrix[0,2] - ori_matrix[2,0]) / (4*w)
    z = (ori_matrix[1,0] - ori_matrix[0,1]) / (4*w)
    return (x, y, z, w)



quat = [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]
quat = np.random.randn(4)
quat = [0, 0, 0, 1.]
quat = np.array(quat)
quat = quat / np.linalg.norm(quat)
r = R.from_quat(quat)
print('scipy result:')
print(r.as_dcm())
print('my result:')
print(quarternion_to_matrix(quat[0], quat[1], quat[2], quat[3]))
matrix = quarternion_to_matrix(quat[0], quat[1], quat[2], quat[3])

print('scipy result:')
print(quat)
print(matrix_to_quarternion(matrix))
