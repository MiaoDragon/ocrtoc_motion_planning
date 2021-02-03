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



# create a database of joint trajectories to initialize IK
database_sz = 20
# read database from file if it exists
import os
import pickle
ik_database_path = '/root/ocrtoc_ws/install/share/motion_planning/data/ik_database.pkl'
if os.path.exists(ik_database_path):
    f = open(ik_database_path, 'r')
    ik_database = pickle.load(f)
    f.close()
else:
    ik_database = None





def fk(state):
    rospy.wait_for_service('compute_fk')
    # generate message
    fk_link_name = ['ee_link']
    fk_robot_state = state
    fk_header = group.get_current_pose().header
    try:
        compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        resp = compute_fk(fk_header, fk_link_name, fk_robot_state)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)
    if resp.error_code.val != 1:
        rospy.logerr("FK service failed with error code: %d" % (resp.error_code.val))
    pose = resp.pose_stamped[0].pose
    return pose

def scale_pose_to_tf(scale=None, pose=None):
    t = np.array([pose.position.x,pose.position.y,pose.position.z])
    r = np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    r = tf.transformations.euler_from_quaternion(r)
    T = tf.transformations.compose_matrix(scale=scale, translate=t, angles=r)
    return T
def tf_to_scale_pose(obj_tf):
    """
    decompose the TF matrix. Return the scale and transformed Pose

    scale: vector of size 3
    pose: geometry_msgs.msg/Pose
    """
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(obj_tf)
    quat = tf.transformations.quaternion_from_euler(*angles)
    p = Pose()
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = trans[2]
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return scale, p

def grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2, grasp_pose):
    """
    function:
    ===========================================
    given start object pose, target object pose, and start grasp pose, obtain the target grasp pose.
    Assuming grasp pose relative to the object does not change.

    input:
    ===========================================
    object_pose1, object_pose2, grasp_pose
    format: geometry_msgs/Pose

    output:
    ===========================================
    target_grasp_pose

    format: geometry_msgs/Pose
    """
    ## TODO: check if scale will affect this. Because the complete pose also includes the scale.
    def object_transformation(object_pose1, object_pose2):
        ## TODO: check if scale will affect this. Because the complete pose also includes the scale.
        # given two poses, compute the transformation
        t1 = np.array([object_pose1.position.x,object_pose1.position.y,object_pose1.position.z])
        r1 = np.array([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
        r1 = tf.transformations.euler_from_quaternion(r1)
        T1 = tf.transformations.compose_matrix(translate=t1, angles=r1)

        t2 = np.array([object_pose2.position.x,object_pose2.position.y,object_pose2.position.z])
        r2 = np.array([object_pose2.orientation.x,object_pose2.orientation.y,object_pose2.orientation.z,object_pose2.orientation.w])
        r2 = tf.transformations.euler_from_quaternion(r2)
        T2 = tf.transformations.compose_matrix(translate=t2, angles=r2)
        T = T2.dot(tf.transformations.inverse_matrix(T1))
        return T
    # obtain transformation matrix of object
    T = object_transformation(obj_pose1, obj_pose2)
    # transform arm pose into the desire one
    target_pose = copy.deepcopy(grasp_pose)
    T_grasp = scale_pose_to_tf(scale=None, pose=target_pose)
    T_target_grasp = T.dot(T_grasp)
    scale, target_pose = tf_to_scale_pose(T_target_grasp)
    return target_pose

def robot_state_from_joint_trajectory(joint_trajectory):
    # obtain the robot state from the planned trajectory
    state = robot.get_current_state()
    state.joint_state.header = joint_trajectory.header
    state.joint_state.name = joint_trajectory.joint_names
    state.joint_state.position = joint_trajectory.points[-1].positions
    state.joint_state.velocity = joint_trajectory.points[-1].velocities
    state.joint_state.effort = joint_trajectory.points[-1].effort
    return state

def robot_state_from_plan(plan):
    # obtain the robot state from the planned trajectory
    state = robot_state_from_joint_trajectory(plan.joint_trajectory)
    return state

def retreat_vec_calculation(pose, local_retreat_vec=np.array([-1.,0.,0.])):
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
    rot_matrix = quarternion_to_matrix(pose.orientation.x, pose.orientation.y, \
                                       pose.orientation.z, pose.orientation.w)
    retreat_vec = rot_matrix.dot(local_retreat_vec)
    return retreat_vec

from std_srvs.srv import Empty
def clear_octomap():
    # update octomap by clearing existing ones
    rospy.loginfo("calling clear_octomap...")
    rospy.wait_for_service('clear_octomap')
    # generate message
    try:
        grasp_gen = rospy.ServiceProxy('clear_octomap', Empty)
        resp1 = grasp_gen()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

from moveit_msgs.msg import DisplayRobotState
def display_robot_state(robot_state, block=True):
    display_msg = DisplayRobotState()
    display_msg.state = robot_state
    robot_state_pub = rospy.Publisher(
        rospy.resolve_name('/display_robot_state'),
        DisplayRobotState, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    robot_state_pub.publish(display_msg)
    print("Publishing display message...")

    if block:
        raw_input("display input...")
        #rospy.sleep(1.0)
    
# IK
from std_msgs.msg import Header, Duration
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionFK

from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity


def get_state_validity(state, group_name="robot_arm"):
    rospy.wait_for_service('/check_state_validity')
    sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    gsvr = GetStateValidityRequest()
    gsvr.robot_state = state
    gsvr.group_name = group_name
    result = sv_srv.call(gsvr)
    # if not result.valid:
    #     print('Collision Checker failed.')
    #     print('contact: ')
    #     for i in range(len(result.contacts)):
    #         print('contact_body_1: %s, type: %d' % (result.contacts[i].contact_body_1, result.contacts[i].body_type_1))
    #         print('contact_body_2: %s, type: %d' % (result.contacts[i].contact_body_2, result.contacts[i].body_type_2))   
    return result

def gripper_collison_free(state, group_name="robot_arm"):
    # only check for self-collision
    #rospy.logwarn('in gripper_collision_free')

    res = get_state_validity(state)
    if res.valid:
        #rospy.logwarn('in gripper_collision_free, return true')
        return True
    for i in range(len(res.contacts)):
        if 'robotiq_2f_85' in res.contacts[i].contact_body_1 or 'robotiq_2f_85' in res.contacts[i].contact_body_2:
            #rospy.logwarn('gripper colliding...')
            return False
    #rospy.logwarn('in gripper_collision_free, return true')
    return True

def self_collision_free(state, group_name="robot_arm"):
    # only check for self-collision
    res = get_state_validity(state)
    if res.valid:
        return True
    for i in range(len(res.contacts)):
        if res.contacts[i].body_type_1 == 0 and res.contacts[i].body_type_2 == 0:
            #print('self contact...')
            return False
    return True

def collision_check_with_model(state, model_name, group_name="robot_arm"):
    res = get_state_validity(state)
    if res.valid:
        return True
    for i in range(len(res.contacts)):
        if not (res.contacts[i].contact_body_1 == model_name and res.contacts[i].contact_body_2 == model_name):
            # other cases, failure
            return False
    return True    
    
def pose_distance(pose1, pose2):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    pos_np = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    ori_np = np.array([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]) 
    target_pos_np = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
    target_ori_np = np.array([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])

    ori_np = tf.transformations.euler_from_quaternion(ori_np)
    target_ori_np = tf.transformations.euler_from_quaternion(target_ori_np)

    ori_np = np.array(ori_np)
    target_ori_np = np.array(target_ori_np)

    pos_dif = pos_np - target_pos_np
    ori_dif = ori_np - target_ori_np
    pos_dif = np.linalg.norm(pos_dif)
    ori_dif = np.linalg.norm(ori_dif, ord=np.inf)
    return pos_dif, ori_dif

def linear_ik(pose, collision, init_pose, init_robot_state, num_waypoints, num_attempts=10):        
    T = scale_pose_to_tf(scale=None, pose=init_pose)
    _, _, init_r, init_pos, _ = tf.transformations.decompose_matrix(T)
    # linearly interpolate
    T = scale_pose_to_tf(scale=None, pose=pose)
    _, _, r, pos, _ = tf.transformations.decompose_matrix(T)
    pos_list = [np.linspace(start=init_pos[i], stop=pos[i], num=num_waypoints) for i in range(len(pos))]
    r_list = [np.linspace(start=init_r[i], stop=r[i], num=num_waypoints) for i in range(len(r))]
    pos_list = np.array(pos_list).T
    r_list = np.array(r_list).T
    # each time use the previous vaild state as initialization
    last_valid_state = init_robot_state
    for i in range(num_waypoints):
        # obtain pose
        T = tf.transformations.compose_matrix(translate=pos_list[i], angles=r_list[i])
        _, current_pose = tf_to_scale_pose(T)
        # ik on this one
        _, robot_state, error_code_val = ik(pose=current_pose, collision=collision, init_robot_state=last_valid_state, num_attempts=num_attempts)
        if error_code_val == 1:
            # update the last_valid_state to the returned state
            last_valid_state = robot_state
    # lastly, compute IK on the current pose
    return ik(pose=pose, collision=collision, init_robot_state=last_valid_state, num_attempts=num_attempts)


def ik(pose, collision=False, init_robot_state=None, num_attempts=50):
    global ik_database
    # verify if the pose is in collision
    #robot = moveit_commander.RobotCommander()
    #group_arm_name = "robot_arm"
    #group = moveit_commander.MoveGroupCommander(group_arm_name)
    SAMPLE_STATE = 1
    CURRENT_STATE = 0
    pos_dif_threshold = .6
    ori_dif_threshold = 360


    if init_robot_state is None:
        # initialize the robot state by linearly interpolate the pose, and compute IK for each position
        # use the last one as the init

        init_robot_state = robot.get_current_state()
        num_waypoints = 20
        init_pose_stamped = group.get_current_pose()
        init_pose = init_pose_stamped.pose
        last_valid_state, robot_state, error_code_val = linear_ik(pose, collision, init_pose, init_robot_state, num_waypoints, num_attempts=20)
        if error_code_val:
            # IK is solved using initial value
            return last_valid_state, robot_state, error_code_val
        
        # if we want collision check too, then check if we can get self-collision-free but in-collision-with-env state
        # if we found this case, then it means we don't need to do sample anymore
        if collision:
            last_valid_state, robot_state, error_code_val = linear_ik(pose, False, init_pose, init_robot_state, num_waypoints, num_attempts=20)
            #rospy.logwarn('turnning on collision in IK for init robot state. error code: %d' % (error_code_val))
            if error_code_val and not gripper_collison_free(robot_state):
                # if gripper in collision, we will fail sample too
                return last_valid_state, robot_state, False
            # otherwise, we can try to get self-collison-free by sample

        # otherwise, current_state may not be a good init value

        if SAMPLE_STATE:
            if ik_database is None:
                # create database
                print('creating IK database...')
                joint_names = group.get_active_joints()
                robot_state = robot.get_current_state()
                # print('joint_names: ')
                # print(joint_names)
                # print('robot_State joint_state:')
                # print(robot_state.joint_state.name)
                robot_states = []
                robot_poses = []
                database_idx = 0
                while True:
                    if database_idx == database_sz:
                        break
                    print('database_idx: %d' % (database_idx))
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
                    # add constraints on the height
                    if pose.position.z <= 0.2:
                        continue
                    # print('robot_state value:')
                    # print(robot_state.joint_state.position)
                    # fk to get pose
                    database_idx += 1
                    robot_poses.append(pose)
                    robot_states.append(copy.deepcopy(robot_state))    

                ik_database = list(zip(robot_poses, robot_states))
                # store this file to DISK
                f = open(ik_database_path, 'w')
                pickle.dump(ik_database, f)
                f.close()
                print('successfully stored IK database.')


            # sample several states, and compute pose. find nearest several points as initialization
            # do linear_ik from those initialization to see if solution exists
            last_valid_state = None

            for i in range(database_sz):
                init_pose = ik_database[i][0]
                init_robot_state = ik_database[i][1]
                num_waypoints = 5
                pos_dif, ori_dif = pose_distance(pose, init_pose)
                #print('pos_dif: %f, ori_dif: %f' % (pos_dif, ori_dif))
                if pos_dif <= pos_dif_threshold and ori_dif <= ori_dif_threshold * np.pi/180:
                    # use this as an initalization and try linear_ik
                    last_valid_state, robot_state, error_code_val = \
                        linear_ik(pose, collision, init_pose, init_robot_state, num_waypoints, num_attempts=5)
                    if error_code_val:
                        # valid IK found, return it
                        #print('sample state IK success.')
                        return last_valid_state, robot_state, error_code_val
            print('sample_state IK failed too.')
            return last_valid_state, robot_state, error_code_val


    # call MoveIt! IK service to compute the robot state from the pose
    rospy.wait_for_service('compute_ik')
    # generate message
    ik_request = PositionIKRequest()
    ik_request.group_name = "robot_arm"
    ik_request.robot_state = init_robot_state
    #ik_request.avoid_collisions = False
    ik_request.avoid_collisions = collision
    pose_stamped = group.get_current_pose()
    pose_stamped.pose = pose
    ik_request.pose_stamped = pose_stamped
    ik_request.timeout = rospy.Duration.from_sec(0.005)  # from kinematics.yaml
    #ik_request.attempts = num_attempts
    try:
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        resp = compute_ik(ik_request)
    except rospy.ServiceException as e:
        #print("Service call failed: %s"%e)
        sys.exit(1)
    robot_state = resp.solution
    error_code = resp.error_code   

    if error_code.val != 1:
        #print('IK with collision [%d] failed.' % (collision))
        return init_robot_state, robot_state, (error_code.val == 1)
    if collision == True:
        init_robot_state = robot_state
        #print('collision IK success')
        return init_robot_state, robot_state, True

    if self_collision_free(robot_state):
        #print('self_collision_free. IK success.')
        init_robot_state = robot_state
        return init_robot_state, robot_state, True

    #print('self_collision. IK fail.')
    # otherwise, fail
    return init_robot_state, robot_state, False


def ur5_ik(pose, collision=False, init_robot_state=None, num_attempts=50):
    from motion_planning.srv import UR5IKService, UR5IKServiceResponse
    rospy.wait_for_service('ur5_ik_service')
    # transform
    #T = [[ 0.04640509, -0.99614275, -0.07447279,  0.01773419],
    #    [ 0.93846331,  0.017932  ,  0.34491311,  0.68655781],
    #    [-0.34224724, -0.0858957 ,  0.93567556, -0.12381387],
    #    [ 0.        ,  0.        ,  0.        ,  1.        ]]
    # [ 0.12204006  0.13517574  2.75518085 -0.40639803 -0.99993009  0.09734306]
    print('query pose:')
    print(pose)
    pose_T = scale_pose_to_tf(None, pose)
    T = tf.transformations.compose_matrix(None, None, np.array([0.,  0., np.pi/2]), np.array([0.14,0.23,0.]), None)
    T = np.array(T)
    pose_T = T.dot(pose_T)
    _, pose = tf_to_scale_pose(pose_T)
    print('transposed pose:')
    print(pose)
    
    # generate message
    try:
        compute_ik = rospy.ServiceProxy('ur5_ik_service', UR5IKService)
        resp = compute_ik(pose)
    except rospy.ServiceException as e:
        #print("Service call failed: %s"%e)
        sys.exit(1)
    error_code = resp.result   
    joint_states = resp.solutions
    if error_code != 1:
        #rospy.logwarn('IK with collision [%d] failed.' % (collision))
        return init_robot_state, None, False
    
    # go over each joint_state, and check collision for them, copy to robot_state
    robot_state = robot.get_current_state()
    name = list(robot_state.joint_state.name)
    position = list(robot_state.joint_state.position)
    
    #name_indices = [0 for i in range(len(joint_states[0].name))]  # joint_state index -> robot_state index
    name_indices = {}  # name -> index in robot_state
    for i in range(len(name)):
        if name[i] in joint_states[0].name:
            name_indices[name[i]] = i

    for joint_state in joint_states:
        # convert to robot state
        robot_state = copy.deepcopy(robot_state)
        name = list(robot_state.joint_state.name)
        position = list(robot_state.joint_state.position)        
        for i in range(len(joint_state.name)):
            idx = name_indices[joint_state.name[i]]
            position[idx] = joint_state.position[i]
        robot_state.joint_state.position = position

        if collision == True:
            # check collision
            res = get_state_validity(robot_state)
            if res.valid:
                init_robot_state = robot_state
                #rospy.logwarn('IK with collision succeed.')
                return init_robot_state, robot_state, True
        else:
            # check self collision
            if self_collision_free(robot_state):
                init_robot_state = robot_state
                #rospy.logwarn('IK with self collision succeed.')
                return init_robot_state, robot_state, True
    #rospy.logwarn('IK failed.')
    return init_robot_state, None, False



def verify_pose(pose, init_robot_state=None):
    # verify if the pose is in collision
    init_robot_state, robot_state, error_code_val = ik(pose, collision=True, init_robot_state=init_robot_state)
    #init_robot_state, robot_state, error_code_val = ur5_ik(pose, collision=True, init_robot_state=init_robot_state)
    # if failed, then ik with collision OFF to get better initialization value
    if error_code_val != 1:
        pass  # not sure if it is a good idea to use Non-collision for init of collision checker
        #init_robot_state, ik_robot_state, ik_error_code = ik(pose, collision=False, init_robot_state=init_robot_state)
        #if ik_error_code.val == 1:
        #    # update initial value
        #    init_robot_state = ik_robot_state
        #else:
        #    # otherwise, keep the initial value unchanged.
        #    pass
    else:
        # update initial value to the returned state
        init_robot_state = robot_state
    return init_robot_state, robot_state, error_code_val  # True if SUCCESS, otherwise IK or CC failed
