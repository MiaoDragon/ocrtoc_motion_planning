import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point

import numpy as np
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
#from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from control_msgs.msg import GripperCommandActionGoal

import moveit_connection
robot = moveit_connection.robot
group = moveit_connection.group
scene = moveit_connection.scene

from motion_planning_utils import ur5_ik

# sample several pose, and match moveit IK with ur5 IK
# create a database of joint trajectories to initialize IK
database_sz = 20
# read database from file if it exists
import os
import pickle
from tqdm import trange, tqdm
ik_database_path = '/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/scripts/ik_database.pkl'
if os.path.exists(ik_database_path):
    f = open(ik_database_path, 'r')
    ik_database = pickle.load(f)
    f.close()
else:
    ik_database = None

sample_size = 512
vars = np.random.rand(sample_size, 3)
lower = np.array([-1., -1., -1.])
scale = np.array([2., 2., 2.])
vars = vars * scale + lower
n_elite = 16

penalty = 1000.

rospy.init_node('tune_ik')

ik_database.append((group.get_current_pose().pose, robot.get_current_state()))
database_sz += 1
for iter in trange(20):
    # initialize variables
    losses = np.zeros(sample_size)

    for j in trange(len(vars)):
        T_ = tf.transformations.compose_matrix(None, None, np.zeros(3), vars[j], None)
        invalid_n = 0
        for i in range(database_sz):
            pose = ik_database[i][0]
            robot_state = ik_database[i][1]
            #T = tf.transformations.quaternion_matrix(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
            angles = np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            angles = tf.transformations.euler_from_quaternion(angles)
            translation = np.array([pose.position.x,pose.position.y,pose.position.z])
            T = tf.transformations.compose_matrix(None, None, angles, translation, None)

            #T_ = tf.transformations.euler_matrix(vars[j][0],vars[j][1],vars[j][2])
            new_T = T.dot(T_)
            _, _, new_angles, new_t, _ = tf.transformations.decompose_matrix(new_T)
            new_q = tf.transformations.quaternion_from_euler(new_angles[0], new_angles[1], new_angles[2])
            new_pose = Pose()
            new_pose.orientation.x = new_q[0]
            new_pose.orientation.y = new_q[1]
            new_pose.orientation.z = new_q[2]
            new_pose.orientation.w = new_q[3]
            new_pose.position.x = new_t[0]
            new_pose.position.y = new_t[1]
            new_pose.position.z = new_t[2]
            _, ur5_robot_state, status = ur5_ik(new_pose, collision=False, init_robot_state=None, num_attempts=50)
            if not status:
                #losses[j] += penalty
                invalid_n += 1
                continue
            # calculate joint difference
            new_joint = np.array(ur5_robot_state.joint_state.position)
            joint = np.array(robot_state.joint_state.position)
            losses[j] += np.linalg.norm(joint - new_joint)
        if invalid_n == database_sz:
            losses[j] += penalty
            
    # after sampling, select elite samples, and estimate mean and variance
    print('iteration %d: ' % (iter))
    print('losses: ')
    print(losses)

    sort_indices = np.argsort(losses)  # small to large
    if losses[sort_indices[0]] == penalty:
        # failed. resample by uniform
        vars = np.random.rand(sample_size, 3)
        lower = np.array([-1., -1., -1.])
        scale = np.array([2., 2., 2.])
        vars = vars * scale + lower
        continue
    # otherwise, select top-k samples, and estimate mean and variance
    elite_vars = vars[sort_indices[:n_elite]]
    var_mean = vars[sort_indices[:n_elite]].mean(axis=0)
    var_std = vars[sort_indices[:n_elite]].std(axis=0)
    # resample by gaussian
    vars = np.random.randn(sample_size, 6) * var_std + var_mean
    # check boundary
    for j in range(len(vars)):
        for i in range(3):
            if vars[j,i] < lower[i]:
                vars[j,i] = lower[i]
            if vars[j,i] > -lower[i]:
                vars[j,i] = -lower[i]

print('result var:')
print(elite_vars)
