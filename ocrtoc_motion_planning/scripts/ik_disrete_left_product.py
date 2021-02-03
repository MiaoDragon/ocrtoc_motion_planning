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
trans_lower = np.array([-0.1, -0.1, -0.1])
trans_scale = np.array([0.2, 0.2, 0.2])
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

for r1 in [0.]:
    for r2 in [0.]:
        for r3 in [np.pi/2]:
            while True:
                trans = np.random.rand(3) * trans_scale + trans_lower
                #trans = np.zeros(3)
                print('current pose:')
                print(pose)



                angles = np.array([r1, r2, r3])
                T = tf.transformations.compose_matrix(None, None, angles, trans, None)
                
                T_ori = tf.transformations.compose_matrix(None, None, np.zeros(3), trans, None)

                _, pose = tf_to_scale_pose(T.dot(T_pose))
                print('transposed pose:')
                print(pose)
                pose_position = np.array([pose.position.x, pose.position.y, pose.position.z])
                #pose_position += trans
                pose.position.x, pose.position.y, pose.position.z = pose_position[0], pose_position[1], pose_position[2]
                _, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
                if not status:
                    continue
                _, ik_pose = tf_to_scale_pose(T_ori.dot(T_pose))
                _, robot_state, status = ik(ik_pose, collision=False, init_robot_state=None, num_attempts=50)
                if not status:
                    continue


                ur5_robot_pose = fk(ur5_robot_state)
                T_ur5_pose = scale_pose_to_tf(None, ur5_robot_pose)

                ur5_position = np.array([ur5_robot_pose.position.x, ur5_robot_pose.position.y, ur5_robot_pose.position.z])
                
                robot_position = np.array([ik_pose.position.x, ik_pose.position.y, ik_pose.position.z])

                angle_vars.append((r1, r2, r3, T_ur5_pose, ur5_robot_state, robot_state, robot_position-ur5_position))
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

print('distance: ')
print(angle_vars[-1])
display_robot_state(angle_vars[-1][4])
display_robot_state(angle_vars[-1][5])



trans_lower = np.array([-0.5, -0.5, -0.5])
trans_scale = np.array([1., 1., 1.])

#t_choices = np.linspace(.1, .3, num=41)
t_choices = np.linspace(-0., -0.04, num=41)

distance = []
pose = group.get_current_pose()
angle_vars = []
#0.135678, 0.226131
for t1 in [0.14]:
    for t2 in [0.23]:
        for t3 in [0.]:
            fail_case = 0
            while True:
                #t3 = np.random.rand(1) * trans_scale + trans_lower
                #t3 = t3[0]
                #trans = np.random.rand(3) * trans_scale + trans_lower
                #angles = np.array([r1, r2, r3])
                trans = np.array([t1, t2, t3])
                T = tf.transformations.compose_matrix(None, None, np.array([0.,0.,np.pi/2]), trans, None)
                _, pose = tf_to_scale_pose(T.dot(T_pose))
                _, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
                if not status:
                    break
                ur5_robot_pose = fk(ur5_robot_state)
                T_ur5_pose = scale_pose_to_tf(None, ur5_robot_pose)
                angle_vars.append((t1, t2, t3, T_ur5_pose, ur5_robot_state))
                _, _, ur5_angles, ur5_tran, _ = tf.transformations.decompose_matrix(T_ur5_pose)
                _, _, pose_angles, pose_tran, _ = tf.transformations.decompose_matrix(T_pose)
                distance.append(np.linalg.norm(ur5_tran-pose_tran))
                break
print('status:')
print(status)
distance = np.array(distance)
i = np.argmin(distance)
print('distance: ')
print(distance[i])
print('variable:')
print(angle_vars[i][-1])

print('vector distance:')
print('%f, %f, %f' %(angle_vars[i][0],angle_vars[i][1],angle_vars[i][2]))
display_robot_state(angle_vars[i][4])


