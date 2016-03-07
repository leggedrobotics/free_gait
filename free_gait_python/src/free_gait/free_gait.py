#! /usr/bin/env python

import rospy
import free_gait_msgs.msg
from tf.transformations import *
import geometry_msgs.msg
import trajectory_msgs.msg
import tf

def load_action_from_file(file_path, source_frame_id = ''):
    import os
    from rosparam import load_file
    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return None
    
    is_adapt = False
    position = [0, 0, 0]
    orientation = [0, 0, 0, 1]
    
    parameters = load_file(file_path)
    if 'adapt_coordinates' in parameters[0][0]:
        adapt_parameters = parameters[0][0]['adapt_coordinates']
        is_adapt = True
        target_frame_id = adapt_parameters['frame']
        if 'pose' in adapt_parameters:
            position = adapt_parameters['pose']['position']
            orientation = adapt_parameters['pose']['orientation']

    if is_adapt:
        (position, orientation) = transform_coordinates(source_frame_id, target_frame_id, position, orientation)

    return parse_action(parameters, position, orientation)


def load_action_from_file_and_transform(file_path, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    from rosparam import load_file
    import os
    
    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return None

    return parse_action(load_file(file_path), position, orientation)


def parse_action(yaml_object, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    goal = free_gait_msgs.msg.ExecuteStepsGoal()
    
    # For each step.
    for step_parameter in yaml_object[0][0]['steps']:

        # Step.
        step_parameter = step_parameter['step']
        step = free_gait_msgs.msg.Step()
        if not step_parameter:
            continue

        for motion_parameter in step_parameter:
            if 'footstep' in motion_parameter:
                step.footstep.append(parse_footstep(motion_parameter['footstep']))
            if 'leg_mode' in motion_parameter:
                step.leg_mode.append(parse_leg_mode(motion_parameter['leg_mode']))
            if 'joint_trajectory' in motion_parameter:
                step.joint_trajectory.append(parse_joint_trajectory(motion_parameter['joint_trajectory']))
            if 'base_auto' in motion_parameter:
                step.base_auto.append(parse_base_auto(motion_parameter['base_auto']))
            if 'base_trajectory' in motion_parameter:
                step.base_trajectory.append(parse_base_trajectory(motion_parameter['base_trajectory']))
        # # Swing data.
        # if 'swing_data' in step_parameter:
        #     for swing_data_parameter in step_parameter['swing_data']:
        #         swing_data = free_gait_msgs.msg.SwingData()
        #
        #         # Name.
        #         swing_data.name = swing_data_parameter['name']
        #         # Type.
        #         if 'type' in swing_data_parameter:
        #             swing_data.type = swing_data_parameter['type']
        #
        #         # Profile.
        #         if 'profile' in swing_data_parameter:
        #             # Profile target.
        #             target_parameter = swing_data_parameter['profile']['target']
        #             swing_data.profile.target.point.x = target_parameter[0]
        #             swing_data.profile.target.point.y = target_parameter[1]
        #             swing_data.profile.target.point.z = target_parameter[2]
        #             # Profile target frame.
        #             if 'target_frame' in swing_data_parameter['profile']:
        #                 swing_data.profile.target.header.frame_id = swing_data_parameter['profile']['target_frame']
        #             # Profile height.
        #             if 'height' in swing_data_parameter['profile']:
        #                 swing_data.profile.height = swing_data_parameter['profile']['height']
        #             # Profile duration.
        #             if 'duration' in swing_data_parameter['profile']:
        #                 swing_data.profile.duration = rospy.Duration(swing_data_parameter['profile']['duration'])
        #             # Profile type.
        #             if 'type' in swing_data_parameter['profile']:
        #                 swing_data.profile.type = swing_data_parameter['profile']['type']
        #
        #         # Trajectory.
        #         if 'foot_trajectory' in swing_data_parameter:
        #             swing_data.foot_trajectory = parse_multi_dof_trajectory(swing_data.name, swing_data_parameter['foot_trajectory'])
        #
        #         # Trajectory.
        #         if 'joint_trajectory' in swing_data_parameter:
        #             swing_data.joint_trajectory = parse_joint_trajectory(swing_data_parameter['joint_trajectory'])
        #
        #         # No touchdown.
        #         if 'no_touchdown' in swing_data_parameter:
        #             swing_data.no_touchdown = swing_data_parameter['no_touchdown']
        #
        #         # Surface normal.
        #         if 'surface_normal' in swing_data_parameter:
        #             normal_parameter = swing_data_parameter['surface_normal']
        #             swing_data.surface_normal.vector.x = normal_parameter[0]
        #             swing_data.surface_normal.vector.y = normal_parameter[1]
        #             swing_data.surface_normal.vector.z = normal_parameter[2]
        #
        #         # Ignore for pose adaptation.
        #         if 'ignore_for_pose_adaptation' in swing_data_parameter:
        #             swing_data.ignore_for_pose_adaptation = swing_data_parameter['ignore_for_pose_adaptation']
        #
        #         step.swing_data.append(swing_data)
        #
        # # Ignore base shift.
        # if 'ignore_base_shift' in step_parameter:
        #     step.ignore_base_shift = step_parameter['ignore_base_shift']
        #
        # # Base shift data.
        # if 'base_shift_data' in step_parameter:
        #     for base_shift_data_parameter in step_parameter['base_shift_data']:
        #
        #         base_shift_data = free_gait_msgs.msg.BaseShiftData()
        #
        #         # Name.
        #         base_shift_data.name = base_shift_data_parameter['name']
        #         # Ignore.
        #         if 'ignore' in base_shift_data_parameter:
        #             base_shift_data.ignore = base_shift_data_parameter['ignore']
        #         # Type.
        #         if 'type' in base_shift_data_parameter:
        #             base_shift_data.type = base_shift_data_parameter['type']
        #
        #         # Profile.
        #         if 'profile' in base_shift_data_parameter:
        #             # Profile target.
        #             if 'target' in base_shift_data_parameter['profile']:
        #                 target_parameter = base_shift_data_parameter['profile']['target']
        #                 # Profile target position.
        #                 if 'position' in target_parameter:
        #                     base_shift_data.profile.target.pose.position.x = target_parameter['position'][0]
        #                     base_shift_data.profile.target.pose.position.y = target_parameter['position'][1]
        #                     base_shift_data.profile.target.pose.position.z = target_parameter['position'][2]
        #                 # Profile target position.
        #                 if 'orientation' in target_parameter:
        #                     if len(target_parameter['orientation']) == 4:
        #                         base_shift_data.profile.target.pose.orientation.x = target_parameter['orientation'][0]
        #                         base_shift_data.profile.target.pose.orientation.y = target_parameter['orientation'][1]
        #                         base_shift_data.profile.target.pose.orientation.z = target_parameter['orientation'][2]
        #                         base_shift_data.profile.target.pose.orientation.w = target_parameter['orientation'][3]
        #                     if len(target_parameter['orientation']) == 3:
        #                         rpy = target_parameter['orientation']
        #                         quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        #                         base_shift_data.profile.target.pose.orientation.x = quaternion[0]
        #                         base_shift_data.profile.target.pose.orientation.y = quaternion[1]
        #                         base_shift_data.profile.target.pose.orientation.z = quaternion[2]
        #                         base_shift_data.profile.target.pose.orientation.w = quaternion[3]
        #             # Profile target frame.
        #             if 'target_frame' in base_shift_data_parameter['profile']:
        #                 base_shift_data.profile.target.header.frame_id = base_shift_data_parameter['profile']['target_frame']
        #             # Profile height.
        #             if 'height' in base_shift_data_parameter['profile']:
        #                 base_shift_data.profile.height = base_shift_data_parameter['profile']['height']
        #             # Profile duration.
        #             if 'duration' in base_shift_data_parameter['profile']:
        #                 base_shift_data.profile.duration = rospy.Duration(base_shift_data_parameter['profile']['duration'])
        #             # Profile type.
        #             if 'type' in base_shift_data_parameter['profile']:
        #                 base_shift_data.profile.type = base_shift_data_parameter['profile']['type']
        #
        #         # Trajectory.
        #         if 'trajectory' in base_shift_data_parameter:
        #             base_shift_data.trajectory = parse_multi_dof_trajectory('base', base_shift_data_parameter['trajectory'])
        #
        #         step.base_shift_data.append(base_shift_data)
        #
        goal.steps.append(step)
    
    # Adapt to local coordinates if desired.
    if not (numpy.array_equal(position, [0, 0, 0]) and numpy.array_equal(orientation, [0, 0, 0, 1])):
        adapt_coordinates(goal, position, orientation)

    # print goal
    return goal


def parse_footstep(yaml_object):
    footstep = free_gait_msgs.msg.Footstep()
    if not yaml_object:
        return footstep
    if 'name' in yaml_object:
        footstep.name = yaml_object['name']
    if 'target' in yaml_object:
        footstep.target = parse_point(yaml_object['target'])
    if 'profile_height' in yaml_object:
        footstep.profile_height = yaml_object['profile_height']
    if 'average_velocity' in yaml_object:
        footstep.average_velocity = yaml_object['average_velocity']
    if 'profile_type' in yaml_object:
        footstep.profile_type = yaml_object['profile_type']
    if 'ignore_contact' in yaml_object:
        footstep.ignore_contact = yaml_object['ignore_contact']
    if 'surface_normal' in yaml_object:
        footstep.surface_normal = parse_vector(yaml_object['surface_normal'])
    if 'ignore_for_pose_adaptation' in yaml_object:
        footstep.ignore_for_pose_adaptation = yaml_object['ignore_for_pose_adaptation']
    return footstep


def parse_leg_mode(yaml_object):
    leg_mode = free_gait_msgs.msg.LegMode()
    if not yaml_object:
        return leg_mode
    if 'name' in yaml_object:
        leg_mode.name = yaml_object['name']
    if 'support_leg' in yaml_object:
        leg_mode.support_leg = yaml_object['support_leg']
    if 'duration' in yaml_object:
        leg_mode.duration = parse_duration(yaml_object['duration'])
    if 'surface_normal' in yaml_object:
        leg_mode.surface_normal = parse_vector(yaml_object['surface_normal'])
    if 'ignore_for_pose_adaptation' in yaml_object:
        leg_mode.ignore_for_pose_adaptation = yaml_object['ignore_for_pose_adaptation']
    return leg_mode


def parse_joint_trajectory(yaml_object):
    joint_trajectory = free_gait_msgs.msg.JointTrajectory()
    if not yaml_object:
        return joint_trajectory
    if 'name' in yaml_object:
        joint_trajectory.name = yaml_object['name']
    if 'trajectory' in yaml_object:
        joint_trajectory.trajectory = parse_joint_trajectories(yaml_object['trajectory'])
    if 'ignore_contact' in yaml_object:
        joint_trajectory.ignore_contact = yaml_object['ignore_contact']
    if 'surface_normal' in yaml_object:
        joint_trajectory.surface_normal = parse_vector(yaml_object['surface_normal'])
    return joint_trajectory


def parse_base_auto(yaml_object):
    base_auto = free_gait_msgs.msg.BaseAuto()
    if not yaml_object:
        return base_auto
    if 'height' in yaml_object:
        base_auto.height = yaml_object['height']
    if 'ignore_timing_of_leg_motion' in yaml_object:
        base_auto.ignore_timing_of_leg_motion = yaml_object['ignore_timing_of_leg_motion']
    if 'average_linear_velocity' in yaml_object:
        base_auto.average_linear_velocity = yaml_object['average_linear_velocity']
    if 'average_angular_velocity' in yaml_object:
        base_auto.average_angular_velocity = yaml_object['average_angular_velocity']
    if 'support_margin' in yaml_object:
        base_auto.support_margin = yaml_object['support_margin']
    return base_auto


def parse_base_trajectory(yaml_object):
    base_trajectory = free_gait_msgs.msg.BaseTrajectory()
    if not yaml_object:
        return base_trajectory
    if 'trajectory' in yaml_object:
        base_trajectory.trajectory = parse_multi_dof_trajectory('base', yaml_object['trajectory'])
    return base_trajectory


def parse_duration(duration):
    return rospy.Duration(duration)


def parse_point(yaml_object):
    point = geometry_msgs.msg.PointStamped()
    if 'frame' in yaml_object:
        point.header.frame_id = yaml_object['frame']
    if 'position' in yaml_object:
        point.point.x = yaml_object['position'][0]
        point.point.y = yaml_object['position'][1]
        point.point.z = yaml_object['position'][2]
    return point


def parse_vector(yaml_object):
    vector = geometry_msgs.msg.Vector3Stamped()
    if 'frame' in yaml_object:
        vector.header.frame_id = yaml_object['frame']
    if 'vector' in yaml_object:
        vector.vector.x = yaml_object['vector'][0]
        vector.vector.y = yaml_object['vector'][1]
        vector.vector.z = yaml_object['vector'][2]
    return vector


def parse_multi_dof_trajectory(joint_name, trajectory):
    output = trajectory_msgs.msg.MultiDOFJointTrajectory()
    output.header.frame_id = trajectory['frame']
    output.joint_names.append(joint_name)
    for knot in trajectory['knots']:
        point = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
        point.time_from_start = rospy.Time(knot['time'])
        transform = geometry_msgs.msg.Transform()
        transform.translation.x = knot['position'][0]
        transform.translation.y = knot['position'][1]
        transform.translation.z = knot['position'][2]
        if 'orientation' in knot:
            if len(knot['orientation']) == 4:
                transform.rotation.x = knot['orientation'][0]
                transform.rotation.y = knot['orientation'][1]
                transform.rotation.z = knot['orientation'][2]
                transform.rotation.w = knot['orientation'][3]
            if len(knot['orientation']) == 3:
                rpy = knot['orientation']
                quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                transform.rotation.x = quaternion[0]
                transform.rotation.y = quaternion[1]
                transform.rotation.z = quaternion[2]
                transform.rotation.w = quaternion[3]
            
        point.transforms.append(transform)
        output.points.append(point)
    
    return output


def parse_joint_trajectories(yaml_object):
    joint_trajectory = trajectory_msgs.msg.JointTrajectory()
    for joint_name in yaml_object['joint_names']:
        joint_trajectory.joint_names.append(joint_name)
    for knot in yaml_object['knots']:
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = rospy.Time(knot['time'])
        if 'positions' in knot:
            point.positions = knot['positions']
        if 'velocities' in knot:
            point.velocities = knot['velocities']
        if 'accelerations' in knot:
            point.accelerations = knot['accelerations']
        if 'effort' in knot:
            point.effort = knot['effort']
        joint_trajectory.points.append(point)
    
    return joint_trajectory


def adapt_coordinates(goal, position, orientation):
    # For each steps.
    translation = translation_matrix(position)
    z_axis = [0, 0, 1]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    rotation = rotation_matrix(yaw, z_axis)
    transform = concatenate_matrices(translation, rotation)

    for step in goal.steps:
        for foostep in step.footstep:
            position = foostep.target.point;
            if check_if_position_valid(position):
                position = transform_position(transform, position)
                foostep.target.point = position
            # if 'base_auto' in motion_parameter:
            #     step.base_auto.append(parse_base_auto(motion_parameter['base_auto']))
            # if 'footstep' in motion_parameter:
            #     step.footstep.append(parse_footstep(motion_parameter['footstep']))


            # for swing_data in step.swing_data:
            #     position = swing_data.profile.target.point;
            #     if check_if_position_valid(position):
            #         position = transform_position(transform, position)
            #         swing_data.profile.target.point = position
            #     for point in swing_data.foot_trajectory.points:
            #         position = transform_position(transform, point.transforms[0].translation)
            #         point.transforms[0].translation = position
            # for base_shift_data in step.base_shift_data:
            #     pose = base_shift_data.profile.target.pose;
            #     if check_if_pose_valid(pose):
            #         pose = transform_pose(transform, pose)
            #         base_shift_data.profile.target.pose = pose
            #     for point in base_shift_data.trajectory.points:
            #         transformation = transform_transformation(transform, point.transforms[0])
            #         point.transforms[0] = transformation


def transform_coordinates(source_frame_id, target_frame_id, position = [0, 0, 0], orientation = [0, 0, 0, 1], listener = None):
    
    if listener is None:
        listener = tf.TransformListener()
        # Not working in current version of tf/tf2.
        # http://answers.ros.org/question/207039/tfexception-thrown-while-using-waitfortransform/
        # listener.waitForTransform(source_frame_id, target_frame_id, rospy.Time(0), rospy.Duration(10.0))
        rospy.sleep(1.0)

    try:
        (translation, rotation) = listener.lookupTransform(source_frame_id, target_frame_id, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not look up TF transformation from "' +
                     source_frame_id + '" to "' + target_frame_id + '".')
        return None

    transformed_position = translation + quaternion_matrix(rotation)[:3, :3].dot(position)
    transformed_orientation = quaternion_multiply(rotation, orientation)
    return transformed_position, transformed_orientation


def get_transform(source_frame_id, target_frame_id, listener = None):
    
    if listener is None:
        listener = tf.TransformListener()
        # Not working in current version of tf/tf2.
        # listener.waitForTransform(source_frame_id, target_frame_id, rospy.Time(0), rospy.Duration(10.0))
        rospy.sleep(1.0)

    try:
        (translation, rotation) = listener.lookupTransform(source_frame_id, target_frame_id, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not look up TF transformation from "' +
                     source_frame_id + '" to "' + target_frame_id + '".')
        return None

    translation_matrix_form = translation_matrix(translation)
    rotation_matrix_form = quaternion_matrix(rotation)
    return concatenate_matrices(translation_matrix_form, rotation_matrix_form)


def transform_position(transform, position):
    transformed_point = transform.dot([position.x, position.y, position.z, 1.0])
    return geometry_msgs.msg.Point(transformed_point[0], transformed_point[1], transformed_point[2])


def transform_orientation(transform, orientation):
    q1 = quaternion_from_matrix(transform)
    q2 = [orientation.x, orientation.y, orientation.z, orientation.w]
    q = quaternion_multiply(q1, q2)
    return geometry_msgs.msg.Quaternion(q[0], q[1], q[2], q[3])


def transform_pose(transform, pose):
    pose.position = transform_position(transform, pose.position)
    pose.orientation = transform_orientation(transform, pose.orientation)
    return pose


def transform_transformation(transform, transformation):
    transformation.translation = transform_position(transform, transformation.translation)
    transformation.rotation = transform_orientation(transform, transformation.rotation)
    return transformation


def check_if_position_valid(position):
    if (position.x == 0 and position.y == 0 and position.z == 0):
        return False
    else:
        return True


def check_if_orientation_valid(orientation):
    if orientation.x == 0 and orientation.y == 0 and orientation.z == 0 and orientation.w == 0:
        return False
    else:
        return True


def check_if_pose_valid(pose):
    if check_if_position_valid(pose.position) and check_if_orientation_valid(pose.orientation):
        return True
    else:
        return False
