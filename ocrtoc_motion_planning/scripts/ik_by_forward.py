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
vars = vars * scale + lower
n_elite = 16

penalty = 1000.

rospy.init_node('tune_ik')

ik_database.append((group.get_current_pose().pose, robot.get_current_state()))
database_sz += 1

pose = group.get_current_pose().pose
robot_state = robot.get_current_state()

for i in range(database_sz*10):
    #pose = ik_database[0][0]
    #state = ik_database[0][1]
    joint_values_i = group.get_random_joint_values()
    # print('joint value: ')
    # print(joint_values_i)
    pos = list(robot_state.joint_state.position)
    for k in range(6):  #6DOF
        pos[k] = joint_values_i[k]
    robot_state.joint_state.position = pos
    # check collision for the state
    if not self_collision_free(robot_state):
        # in collision
        continue
    pose = fk(robot_state)

    _, ur5_robot_state, status = ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50)
    if status:
        ur5_robot_pose = fk(ur5_robot_state)
        # pose = ur5_robot_pose * T
        # T = ur5_robot_pose^{-1} * pose
        T_pose = scale_pose_to_tf(None, pose)
        T_ur5 = scale_pose_to_tf(None, ur5_robot_pose)
        T = tf.transformations.inverse_matrix(T_ur5).dot(T_pose)
        print(T)
        break