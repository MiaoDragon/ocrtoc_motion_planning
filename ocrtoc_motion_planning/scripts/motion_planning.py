#! /usr/bin/env python

"""
motion planning components:

specify several functions to be used for motion planning

"""

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

#======================================================================================================
#========================================  Tool Box  ==================================================
robot = moveit_commander.RobotCommander()
group_arm_name = "robot_arm"
group = moveit_commander.MoveGroupCommander(group_arm_name)
scene = moveit_commander.PlanningSceneInterface()

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


def robot_state_from_plan(plan):
    robot = moveit_commander.RobotCommander()
    # obtain the robot state from the planned trajectory
    state = robot.get_current_state()
    state.joint_state.header = plan.joint_trajectory.header
    state.joint_state.name = plan.joint_trajectory.joint_names
    state.joint_state.position = plan.joint_trajectory.points[-1].positions
    state.joint_state.velocity = plan.joint_trajectory.points[-1].velocities
    state.joint_state.effort = plan.joint_trajectory.points[-1].effort
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
#======================================================================================================
#======================================================================================================
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
        init_robot_state = robot.get_current_state()
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
    ik_request.attempts = 500
    try:
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        resp = compute_ik(ik_request)
        print('ik_request result:')
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1) 
    robot_state = resp.solution
    error_code = resp.error_code   

    if error_code.val != 1:
        print("IK service failed with error code: %d" % (error_code.val))
        
    return robot_state, error_code

def verify_pose(pose):
    # verify if the pose is in collision
    #robot = moveit_commander.RobotCommander()
    #group_arm_name = "robot_arm"
    #group = moveit_commander.MoveGroupCommander(group_arm_name)
    #print('=========computing IK without collision========')
    #robot_state = ik(pose, collision=False)
    #print('=========computing IK with collision========')
    robot_state, error_code = ik(pose, collision=True)
    return (error_code.val == 1)  # True if SUCCESS, otherwise IK or CC failed

def gripper_openning():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    # publish this so that gazebo can move accordingly
    # Example: control ur5e by topic
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = 0.
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for openning")
    rospy.sleep(1.0)

def gripper_closing():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    # publish this so that gazebo can move accordingly
    # Example: control ur5e by topic
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = 0.6
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for closing")
    rospy.sleep(1.0)


def one_shot_grasp_with_object_pose_attach_object(model_name, scale, obj_pose1, obj_pose2):
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    <-- execute gripper closing -->
    4. Attach the object to the gripper. plan a path to drop pose
    <-- execute gipper openning -->

    input:
    ===========================================
    model_name, obj_pose1, obj_pose2

    format: geometry_msgs/Pose

    output:
    ===========================================
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory

    format: geometry_msgs/Pose
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())


    #** stage 1: generate grasp pose proposals **
    from grasp_srv.msg import ObjectPoses, Grasps
    from grasp_srv.srv import GraspGen, GraspGenResponse
    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    grasp_srv_request_poses.object_scales.append(scale[0])
    grasp_srv_request_poses.object_poses.append(obj_pose1)
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        resp1 = grasp_gen(grasp_srv_request_poses)
        print('pose generation result:')
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

    # select pose for use based on score
    ## TODO: search if we can do multi-goal planning
    grasp_poses = resp1.grasps.global_grasp_poses[0].grasp_poses
    pre_grasp_poses = resp1.grasps.global_grasp_poses[0].pre_grasp_poses

    grasp_pose = grasp_poses[0]
    pre_grasp_pose = pre_grasp_poses[0]

    print("============ Printing generated pre-grasp pose")
    print(pre_grasp_pose)

    print('============ openning gripper...')
    gripper_openning()
    group.clear_pose_targets()

    #** stage 2: generate pre-grasp plan **
    print('============ move arm...')

    # TODO:
    # we can also add a warm-up phase to plan to the intermediate point, in order to obtain a better seed for IK.
    # another way is to generate a continous waypoint trajectory connecting current position to the goal, and obtain
    # IK for each place. We will use the furthest point as the seed for IK.

    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    retreat_vec = retreat_vec_calculation(pre_grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))

    retreat_step_size = 0.001
    current_retreat_step = 0.
    x = group.get_current_pose()

    for planning_attempt_i in range(100):
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = pre_grasp_pose_np + current_retreat_step * retreat_vec
        pre_grasp_pose.position.x = current_pose[0]
        pre_grasp_pose.position.y = current_pose[1]
        pre_grasp_pose.position.z = current_pose[2]
        # verify if the pose is valid by IK and Collision Check
        status = verify_pose(pre_grasp_pose)
        if not status:
            pre_grasp_plan = None
            print('Goal pose is not valid. Another attempt is tried...')
            continue
        x.pose = pre_grasp_pose

        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)

        # set constraint
        """
        path_constraint = Constraints()
        path_constraint.name = "boundary_constraint"
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = robot.get_planning_frame()
        position_constraint.link_name = group.get_end_effector_link()
        #position_constrint.target_point_offset =

        # define the bounding volume of the constraint as a box around the table
        constraint_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = 1
        table_x_max = 0.8
        table_y_max = 0.8
        z_max = 0.4
        primitive.dimensions = [table_x_max*2, table_y_max*2, z_max*2]
        constraint_region.primitives.append(primitive)
        position_constraint.constraint_region = constraint_region
        path_constraint.position_constraints.append(position_constraint)
        group.set_path_constraints(path_constraint)
        """
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            #print('plan is successful')
            pre_grasp_plan = plan
            break
        else:
            pre_grasp_plan = None
    
    pre_grasp_trajectory = pre_grasp_plan.joint_trajectory
    # remember the end state to be used for next stage
    pre_grasp_plan_end_state = robot_state_from_plan(pre_grasp_plan)
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    #** stage 3: plan cartesian path to grasp pose **
    # directly using IK to achieve this part

    # firstly generate list of waypoints from pre_grasp_pose to grasp_pose, until collision happens
    # use around 20 points in between
    robot_state, error_code = ik(pre_grasp_pose, collision=False, init_robot_state=pre_grasp_plan_end_state)
    num_waypoints = 20
    total_duration = 2.  # we want 2s to go to the desired location
    time_step = total_duration / num_waypoints
    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    step = (grasp_pose_np - pre_grasp_pose_np) / num_waypoints
    waypoint_list = []
    pre_to_grasp_trajectory = JointTrajectory()
    pre_to_grasp_trajectory.header = Header()
    #pre_to_grasp_trajectory.header.stamp = rospy.Time.now()
    pre_to_grasp_trajectory.header.frame_id = pre_grasp_trajectory.header.frame_id
    pre_to_grasp_trajectory.joint_names = pre_grasp_trajectory.joint_names

    last_valid_state = pre_grasp_plan_end_state
    for i in range(1,num_waypoints+1):
        inter_pose_np = pre_grasp_pose_np + step * i
        inter_pose = copy.deepcopy(pre_grasp_pose)
        inter_pose.position.x = inter_pose_np[0]
        inter_pose.position.y = inter_pose_np[1]
        inter_pose.position.z = inter_pose_np[2]
        robot_state, error_code = ik(inter_pose, collision=True, init_robot_state=last_valid_state)
        if error_code.val == 1:
            # if IK is successful
            # append robot_state into trajectory
            inter_joint_traj_point = JointTrajectoryPoint()
            inter_joint_traj_point.positions = robot_state.joint_state.position[:6]  # filter out the gripper joints
            inter_joint_traj_point.velocities = robot_state.joint_state.velocity[:6]
            #inter_joint_traj_point.accelerations = 0.
            inter_joint_traj_point.effort = robot_state.joint_state.effort[:6]
            inter_joint_traj_point.time_from_start = rospy.Duration.from_sec(time_step * i)
            pre_to_grasp_trajectory.points.append(inter_joint_traj_point)

            last_valid_state = robot_state
        else:
            print("IK failed at straight-line")
            
    grasp_plan_end_state = last_valid_state

    """
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    retreat_vec = retreat_vec_calculation(grasp_pose, local_retreat_vec=np.array([-1.,0,0]))
    retreat_step_size = 0.001
    current_retreat_step = 0.
    for planning_attempt_i in range(100):
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = grasp_pose_np + current_retreat_step * retreat_vec
        grasp_pose.position.x = current_pose[0]
        grasp_pose.position.y = current_pose[1]
        grasp_pose.position.z = current_pose[2]
        # verify if the pose is valid by IK and Collision Check
        status = verify_pose(grasp_pose)
        if not status:
            pre_to_grasp_plan = None
            print('Goal pose is not valid. Another attempt is tried...')
            continue

        x.pose = grasp_pose
        group.set_start_state(pre_grasp_plan_end_state)  # set the start state as the last state of previous plan
        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            pre_to_grasp_plan = plan
            break
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    
    # define grasp_plan_end_state to connect plan
    grasp_plan_end_state = robot_state_from_plan(pre_to_grasp_plan)
    pre_to_grasp_trajectory = pre_to_grasp_plan.joint_trajectory
    """

    #



    #** stage 4: attach the object to the gripper. plan the place trajectory to move the arm **
    #** attach object to gripper **
    # achieve this by modifying the start robot state
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
    from geometry_msgs.msg import Point
    import pyassimp
    attached_obj = AttachedCollisionObject()
    attached_obj.link_name = "robotiq_2f_85_left_pad"
    touch_links = ["robotiq_2f_85_left_pad", "robotiq_2f_85_right_pad", \
                   "robotiq_2f_85_left_spring_link", "robotiq_2f_85_right_spring_link", \
                   "robotiq_2f_85_left_follower", "robotiq_2f_85_right_follower", \
                   "robotiq_2f_85_left_driver", "robotiq_2f_85_right_driver"]
    attached_obj.touch_links = touch_links
    # specify the attached object shape
    attached_obj_shape = CollisionObject()
    attached_obj_shape.id = model_name
    mesh = Mesh()
    # load mesh file
    pyassimp_mesh = pyassimp.load("/root/ocrtoc_materials/models/"+model_name+"/meshes/textured.obj")
    if not pyassimp_mesh.meshes:
        rospy.logerr('Unable to load mesh')
        sys.exit(1)
    for face in pyassimp_mesh.meshes[0].faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = [face[0],
                                   face[1],
                                   face[2]]
        mesh.triangles.append(triangle)
    for vertex in pyassimp_mesh.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[0]
        point.z = vertex[2]*scale[0]
        mesh.vertices.append(point)
    pyassimp.release(pyassimp_mesh)
    attached_obj_shape.meshes.append(mesh)
    attached_obj_shape.mesh_poses.append(obj_pose1)
    attached_obj_shape.operation = attached_obj_shape.ADD
    print('robot group header:')
    print(group.get_current_pose().header)
    attached_obj_shape.header = group.get_current_pose().header

    attached_obj.object = attached_obj_shape
    grasp_plan_end_state.attached_collision_objects.append(attached_obj)
    #** plan **
    target_pose = grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2, grasp_pose)
    place_plan = place(start_state=grasp_plan_end_state, target_pose=target_pose)
    place_trajectory = place_plan.joint_trajectory
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize
    
    # remove the object from the planning scene
    scene.remove_attached_object(link="robotiq_2f_85_left_pad", name=model_name)

    return pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory



def one_shot_grasp_with_object_pose(model_name, scale, obj_pose1, obj_pose2):
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    <-- execute gripper closing -->
    4. plan a path to drop pose
    <-- execute gipper openning -->

    input:
    ===========================================
    model_name, obj_pose1, obj_pose2

    format: geometry_msgs/Pose

    output:
    ===========================================
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory

    format: geometry_msgs/Pose
    """
    #robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #group_arm_name = "robot_arm"
    #group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())


    #** stage 1: generate grasp pose proposals **
    from grasp_srv.msg import ObjectPoses, Grasps
    from grasp_srv.srv import GraspGen, GraspGenResponse
    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    grasp_srv_request_poses.object_scales.append(scale[0])
    grasp_srv_request_poses.object_poses.append(obj_pose1)
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        resp1 = grasp_gen(grasp_srv_request_poses)
        print('pose generation result:')
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

    # select pose for use based on score
    ## TODO: search if we can do multi-goal planning
    grasp_poses = resp1.grasps.global_grasp_poses[0].grasp_poses
    pre_grasp_poses = resp1.grasps.global_grasp_poses[0].pre_grasp_poses

    grasp_pose = grasp_poses[0]
    pre_grasp_pose = pre_grasp_poses[0]

    print("============ Printing generated pre-grasp pose")
    print(pre_grasp_pose)

    #print('============ openning gripper...')
    #gripper_openning()
    #hello = raw_input("please input\n")
    #rospy.sleep(1.0) # allow publisher to initialize


    group.clear_pose_targets()

    #** stage 2: generate pre-grasp plan **
    print('============ move arm...')

    # TODO:
    # we can also add a warm-up phase to plan to the intermediate point, in order to obtain a better seed for IK.
    # another way is to generate a continous waypoint trajectory connecting current position to the goal, and obtain
    # IK for each place. We will use the furthest point as the seed for IK.

    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    retreat_vec = retreat_vec_calculation(pre_grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))

    retreat_step_size = 0.001
    current_retreat_step = 0.
    x = group.get_current_pose()
    for planning_attempt_i in range(100):
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = pre_grasp_pose_np + current_retreat_step * retreat_vec
        pre_grasp_pose.position.x = current_pose[0]
        pre_grasp_pose.position.y = current_pose[1]
        pre_grasp_pose.position.z = current_pose[2]
        # verify if the pose is valid by IK and Collision Check
        status = verify_pose(pre_grasp_pose)
        if not status:
            pre_grasp_plan = None
            print('Goal pose is not valid. Another attempt is tried...')
            continue
        x.pose = pre_grasp_pose

        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)

        # set constraint
        """
        path_constraint = Constraints()
        path_constraint.name = "boundary_constraint"
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = robot.get_planning_frame()
        position_constraint.link_name = group.get_end_effector_link()
        #position_constrint.target_point_offset =

        # define the bounding volume of the constraint as a box around the table
        constraint_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = 1
        table_x_max = 0.8
        table_y_max = 0.8
        z_max = 0.4
        primitive.dimensions = [table_x_max*2, table_y_max*2, z_max*2]
        constraint_region.primitives.append(primitive)
        position_constraint.constraint_region = constraint_region
        path_constraint.position_constraints.append(position_constraint)
        group.set_path_constraints(path_constraint)
        """
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            #print('plan is successful')
            pre_grasp_plan = plan
            break
        else:
            pre_grasp_plan = None
    
    pre_grasp_trajectory = pre_grasp_plan.joint_trajectory
    # remember the end state to be used for next stage
    pre_grasp_plan_end_state = robot_state_from_plan(pre_grasp_plan)
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    #** stage 3: plan cartesian path to grasp pose **
    # directly using IK to achieve this part

    # firstly generate list of waypoints from pre_grasp_pose to grasp_pose, until collision happens
    # use around 20 points in between
    robot_state, error_code = ik(pre_grasp_pose, collision=False, init_robot_state=pre_grasp_plan_end_state)
    num_waypoints = 20
    total_duration = 2.  # we want 2s to go to the desired location
    time_step = total_duration / num_waypoints
    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    step = (grasp_pose_np - pre_grasp_pose_np) / num_waypoints
    waypoint_list = []
    pre_to_grasp_trajectory = JointTrajectory()
    pre_to_grasp_trajectory.header = Header()
    #pre_to_grasp_trajectory.header.stamp = rospy.Time.now()
    pre_to_grasp_trajectory.header.frame_id = pre_grasp_trajectory.header.frame_id
    pre_to_grasp_trajectory.joint_names = pre_grasp_trajectory.joint_names

    last_valid_state = pre_grasp_plan_end_state
    for i in range(1,num_waypoints+1):
        inter_pose_np = pre_grasp_pose_np + step * i
        inter_pose = copy.deepcopy(pre_grasp_pose)
        inter_pose.position.x = inter_pose_np[0]
        inter_pose.position.y = inter_pose_np[1]
        inter_pose.position.z = inter_pose_np[2]
        robot_state, error_code = ik(inter_pose, collision=True, init_robot_state=last_valid_state)
        if error_code.val == 1:
            # if IK is successful
            # append robot_state into trajectory
            inter_joint_traj_point = JointTrajectoryPoint()
            inter_joint_traj_point.positions = robot_state.joint_state.position[:6]  # filter out the gripper joints
            inter_joint_traj_point.velocities = robot_state.joint_state.velocity[:6]
            #inter_joint_traj_point.accelerations = 0.
            inter_joint_traj_point.effort = robot_state.joint_state.effort[:6]
            inter_joint_traj_point.time_from_start = rospy.Duration.from_sec(time_step * i)
            pre_to_grasp_trajectory.points.append(inter_joint_traj_point)

            last_valid_state = robot_state
        else:
            print("IK failed at straight-line")
            
    grasp_plan_end_state = last_valid_state

    """
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    retreat_vec = retreat_vec_calculation(grasp_pose, local_retreat_vec=np.array([-1.,0,0]))
    retreat_step_size = 0.001
    current_retreat_step = 0.
    for planning_attempt_i in range(100):
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = grasp_pose_np + current_retreat_step * retreat_vec
        grasp_pose.position.x = current_pose[0]
        grasp_pose.position.y = current_pose[1]
        grasp_pose.position.z = current_pose[2]
        # verify if the pose is valid by IK and Collision Check
        status = verify_pose(grasp_pose)
        if not status:
            pre_to_grasp_plan = None
            print('Goal pose is not valid. Another attempt is tried...')
            continue

        x.pose = grasp_pose
        group.set_start_state(pre_grasp_plan_end_state)  # set the start state as the last state of previous plan
        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            pre_to_grasp_plan = plan
            break
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    
    # define grasp_plan_end_state to connect plan
    grasp_plan_end_state = robot_state_from_plan(pre_to_grasp_plan)
    pre_to_grasp_trajectory = pre_to_grasp_plan.joint_trajectory
    """
    #** stage 4: plan the place trajectory to move the arm **
    target_pose = grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2, grasp_pose)
    place_plan = place(start_state=grasp_plan_end_state, target_pose=target_pose)
    place_trajectory = place_plan.joint_trajectory
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    return pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory


def execute_plan(pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory):
    # ** execution of plan **
    # execute plan for grasping
    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = pre_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    arm_cmd = pre_to_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    print('============ closing gripper...')
    gripper_closing()

    # execute plan for placing
    hello = raw_input("please input\n")
    rospy.sleep(2)
    arm_cmd = place_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    # open gripper
    print('=========== openning gripper...')
    gripper_openning()


def place(start_state, target_pose):
    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    #group_arm_name = "robot_arm"
    #group = moveit_commander.MoveGroupCommander(group_arm_name)
    group.clear_pose_targets()
    # verify if the target pose is collision free
    # status = verify_pose(target_pose)
    # if not status:
    #     place_plan = None
    #     print('Goal pose is not valid. Another attempt is tried...')
    #     continue
    # plan
    for planning_attempt_i in range(10):
        group.set_start_state(start_state)
        group.set_pose_target(target_pose)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            place_plan = plan
            break
        else:
            place_plan = None
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0
    return place_plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_test',
                    anonymous=True)
    import json
    scene_pose = open('../scene_info/1-1.json', 'r')
    scene_pose = json.load(scene_pose)
    # jenga
    #data_str = '-0.0599994957447052 -0.31989747285842896 0.014999995008111 2.261107306891991e-09 -1.8113155351356218e-09 -6.071384286259734e-09'
    # potted_meat_can
    #model_name = 'potted_meat_can'
    #data_str = '0.15989847481250763 0.29000547528266907 0.04212800785899162 -4.840692449249814e-08 -4.5454357156415357e-07 -6.927437318496676e-08'

    # master chef can
    #model_name = 'master_chef_can'
    #data_str = '-0.03987777605652809 0.32008427381515503 0.03530200198292732 -1.4431422264283453e-06 -7.782947982250227e-07 -3.5607089180211774e-07'

    # wood_block
    model_name = 'wood_block'
    data_str = '0.09994173049926758 -0.31943440437316895 0.022629257291555405 -0.005208467812430025 0.005718712075542679 0.005303920438713571'
 
    # pudding_box
    #model_name = 'pudding_box'
    #data_str = '0.25974133610725403 -0.3102521002292633 0.009862901642918587 0.022348699698035056 -0.01139162625176719 -9.36981416203137e-05'

    # find the model_name in the scene_pose
    for i in range(len(scene_pose)):
        if scene_pose[i]['name'] == model_name:
            # record the pose
            start_tf = np.array(scene_pose[i]['pose_world'])
            scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(start_tf)
            break

    # parsing the string
    data = data_str.split()
    for i in range(len(data)):
        data[i] = float(data[i])

    data = np.array(data)
    position = data[:3]
    orientation = data[3:]

    # compose into TF matrix
    target_tf = tf.transformations.compose_matrix(scale=scale, shear=shear, angles=orientation, translate=position, perspective=persp)
    scale, start_pose = tf_to_scale_pose(start_tf)
    scale, target_pose = tf_to_scale_pose(target_tf)
    #pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory = one_shot_grasp_with_object_pose_attach_object(model_name, scale, start_pose, target_pose)
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory = one_shot_grasp_with_object_pose(model_name, scale, start_pose, target_pose)
    execute_plan(pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory)

if __name__ == "__main__":
    main()
