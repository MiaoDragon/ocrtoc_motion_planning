import numpy as np
#from scipy.spatial.transform import Rotation as R
import tf
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point

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

def object_transformation(object_pose1, object_pose2):
    # given two poses, compute the transformation
    R1 = quaternion_matrix([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
    R2 = quaternion_matrix([object_pose2.orientation.x,object_pose2.orientation.y,object_pose2.orientation.z,object_pose2.orientation.w])
    R = R2.dot(tf.transformations.inverse_matrix(R1))
    angles = tf.transformations.euler_from_matrix(R)
    trans = np.array([object_pose2.position.x - object_pose1.position.x, object_pose2.position.y - object_pose1.position.y, object_pose2.position.z - object_pose1.position.z])
    M = tf.transformations.compose_matrix(angles=angles, translate=trans)
    return M

# object transformation
object_pose1 = Pose()
object_pose1.position.x = 0.
object_pose1.position.y = 0.
object_pose1.position.z = 0.
object_pose1.orientation.x = 0.
object_pose1.orientation.y = 0.
object_pose1.orientation.z = 0.
object_pose1.orientation.w = 1.
R1 = tf.transformations.quaternion_matrix([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
t1 = np.array([object_pose1.position.x,object_pose1.position.y,object_pose1.position.z])
r1 = np.array([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
T1 = tf.transformations.compose_matrix(translate=t1, angles=r1)

object_pose2 = Pose()
object_pose2.position.x = 0.3
object_pose2.position.y = 0.4
object_pose2.position.z = 0.5

quar = tf.transformations.quaternion_from_euler(0.3, 0., 0.)
object_pose2.orientation.x = quar[0]
object_pose2.orientation.y = quar[1]
object_pose2.orientation.z = quar[2]
object_pose2.orientation.w = quar[3]
t2 = np.array([object_pose2.position.x,object_pose2.position.y,object_pose2.position.z])
r2 = np.array([0.3, 0., 0.])
T2 = tf.transformations.compose_matrix(translate=t2, angles=r2)

T = T2.dot(tf.transformations.inverse_matrix(T1))


scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(T)
print("T angles:")
print(angles)
print("T trans:")
print(trans)

object_pose3 = Pose()
object_pose3.position.x = 1.
object_pose3.position.y = 2.
object_pose3.position.z = 3.

quar = tf.transformations.quaternion_from_euler(.2,0,0)
object_pose3.orientation.x = quar[0]
object_pose3.orientation.y = quar[1]
object_pose3.orientation.z = quar[2]
object_pose3.orientation.w = quar[3]

t3 = np.array([object_pose3.position.x,object_pose3.position.y,object_pose3.position.z])
r3 = np.array([0., 0, 0])
T3 = tf.transformations.compose_matrix(translate=t3, angles=r3)

T3_ = T.dot(T3)
scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(T3_)
print("scale:")
print(scale)
#R = tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(*angles), quar)
print("using transformation on object 3:")
#R = tf.transformations.quaternion_matrix(R)
print("T3_ angles:")
print(angles)
print("T3_ trans:")
print(trans)
