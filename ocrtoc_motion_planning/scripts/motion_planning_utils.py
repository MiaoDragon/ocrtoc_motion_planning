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
def display_robot_state(robot_state):
    display_msg = DisplayRobotState()
    display_msg.state = robot_state
    robot_state_pub = rospy.Publisher(
        rospy.resolve_name('/display_robot_state'),
        DisplayRobotState, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    robot_state_pub.publish(display_msg)
    print("Publishing display message...")
    rospy.sleep(1.0)
    
# IK
from std_msgs.msg import Header, Duration
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

def ik(pose, collision=False, init_robot_state=None):
    # verify if the pose is in collision
    #robot = moveit_commander.RobotCommander()
    #group_arm_name = "robot_arm"
    #group = moveit_commander.MoveGroupCommander(group_arm_name)
    if init_robot_state is None:
        # initialize the robot state by linearly interpolate the pose, and compute IK for each position
        # use the last one as the init
        init_robot_state = robot.get_current_state()
        num_waypoints = 10
        init_pose_stamped = group.get_current_pose()
        init_pose = init_pose_stamped.pose
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
            _, robot_state, error_code = ik(pose=current_pose, collision=collision, init_robot_state=last_valid_state)
            if error_code.val == 1:
                # update the last_valid_state to the returned state
                last_valid_state = robot_state
            else:
                # failed, try next one
                pass
        # update init_robot_state at the end
        init_robot_state = last_valid_state
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
    ik_request.attempts = 50
    try:
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        resp = compute_ik(ik_request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1) 
    robot_state = resp.solution
    error_code = resp.error_code   

    if error_code.val != 1:
        pass
        #print("IK service failed with error code: %d" % (error_code.val))
    
    # return the point to use for initalization (valid one), and other solutions
    # if IK found, then update init_robot_state to this latest one
    if error_code.val == 1:
        init_robot_state = robot_state
    return init_robot_state, robot_state, error_code

def verify_pose(pose, init_robot_state=None):
    # verify if the pose is in collision
    init_robot_state, robot_state, error_code = ik(pose, collision=True, init_robot_state=init_robot_state)
    # if failed, then ik with collision OFF to get better initialization value
    if error_code.val != 1:
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
    return init_robot_state, robot_state, (error_code.val == 1)  # True if SUCCESS, otherwise IK or CC failed
