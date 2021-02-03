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

from motion_planning_utils import *

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
vars = np.random.rand(sample_size, 6)
lower = np.array([-np.pi, -np.pi, -np.pi, -1., -1., -1.])
scale = np.array([2*np.pi, 2*np.pi, 2*np.pi, 2., 2., 2.])
trans_lower = np.array([-1., -1., -1])
trans_scale = np.array([2., 2., 2.])
vars = vars * scale + lower
n_elite = 16

penalty = 1000.

rospy.init_node('tune_ik')

ik_database.append((group.get_current_pose().pose, robot.get_current_state()))
database_sz += 1

pose = group.get_current_pose().pose
robot_state = robot.get_current_state()
T_pose = scale_pose_to_tf(None, pose)

angle_choices = np.array([0., np.pi/2, np.pi, np.pi/2*3])
distance = []
angle_vars = []
for r1 in angle_choices:
    for r2 in angle_choices:
        for r3 in angle_choices:
            while True:
                trans = np.random.rand(3) * trans_scale + trans_lower
                angles = np.array([r1, r2, r3])
                T = tf.transformations.compose_matrix(None, None, angles, trans, None)
                _, pose = tf_to_scale_pose(T_pose.dot(T))
                _, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
                if not status:
                    continue
                ur5_robot_pose = fk(ur5_robot_state)
                T_ur5_pose = scale_pose_to_tf(None, ur5_robot_pose)
                angle_vars.append((r1, r2, r3, T_ur5_pose, ur5_robot_state))
                _, _, ur5_angles, ur5_tran, _ = tf.transformations.decompose_matrix(T_ur5_pose)
                _, _, pose_angles, pose_tran, _ = tf.transformations.decompose_matrix(T_pose)
                print('ur5_angles: ')
                print(ur5_angles)
                print('pose_angles: ')
                print(pose_angles)
                ur5_angles = np.array(ur5_angles)
                pose_angles = np.array(pose_angles)
                distance.append(np.linalg.norm(ur5_angles-pose_angles))
                break
distance = np.array(distance)
i = np.argmin(distance)
print('distance: ')
print(distance[i])
print('variable:')
print(angle_vars[i])

print('vector distance:')
print(ur5_tran - pose_tran)
display_robot_state(angle_vars[i][4])



# try to determine the translation
sample_size = 1024
vars = np.random.rand(sample_size, 3)
lower = np.array([-1.5, -1.5, -1.5])
scale = np.array([3., 3., 3.])
vars = vars * scale + lower
n_elite = 128
penalty = 1000.
for iter in trange(20):
    losses = np.zeros(sample_size)
    for i in trange(sample_size):
        trans = vars[i]
        T = tf.transformations.compose_matrix(None, None, np.zeros(3), trans, None)
        _, pose = tf_to_scale_pose(T_pose.dot(T))
        _, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
        if not status:
            losses[i] = penalty
            continue
        ur5_robot_pose = fk(ur5_robot_state)
        T_ur5_pose = scale_pose_to_tf(None, ur5_robot_pose)
        _, _, ur5_angles, ur5_tran, _ = tf.transformations.decompose_matrix(T_ur5_pose)
        _, _, pose_angles, pose_tran, _ = tf.transformations.decompose_matrix(T_pose)
        # v_fk = R_1v_2 + v_1
        ur5_tran = np.array(ur5_tran)
        pose_tran = np.array(pose_tran)
        losses[i] = np.linalg.norm(pose_tran - ur5_tran)


    # after sampling, select elite samples, and estimate mean and variance
    print('iteration %d: ' % (iter))


    sort_indices = np.argsort(losses)  # small to large
    print('losses: ')
    print(losses[sort_indices[:n_elite]])

    if losses[sort_indices[0]] == penalty:
        # failed. resample by uniform
        vars = np.random.rand(sample_size, 3)
        lower = np.array([-1.5, -1.5, -1.5])
        scale = np.array([3., 3., 3.])
        vars = vars * scale + lower
        continue
    # otherwise, select top-k samples, and estimate mean and variance
    elite_vars = vars[sort_indices[:n_elite]]
    var_mean = vars[sort_indices[:n_elite]].mean(axis=0)
    var_std = vars[sort_indices[:n_elite]].std(axis=0)
    print('elite variables:')
    print(elite_vars)
    # resample by gaussian
    vars = np.random.randn(sample_size, 3) * var_std + var_mean
    # check boundary
    for j in range(len(vars)):
        for i in range(3):
            if vars[j,i] < lower[i]:
                vars[j,i] = lower[i]
            if vars[j,i] > -lower[i]:
                vars[j,i] = -lower[i]

print('vars:')
print(elite_vars)

T = tf.transformations.compose_matrix(None, None, np.zeros(3), elite_vars[0], None)
_, pose = tf_to_scale_pose(T_pose.dot(T))
_, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
ur5_robot_pose = fk(ur5_robot_state)
display_robot_state(ur5_robot_state)
