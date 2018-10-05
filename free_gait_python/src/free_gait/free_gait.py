#! /usr/bin/env python

import rospy
import rospkg
import free_gait_msgs.msg
from tf.transformations import *
import geometry_msgs.msg
import trajectory_msgs.msg
import tf2_ros

from tf2_msgs.msg import TFMessage # For local LocalTransformListener.


def get_package_path(package):
    rospack = rospkg.RosPack()
    return rospack.get_path(package)


def load_action_from_file(file_path, placeholders=None):
    import os
    from rosparam import load_file
    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return None

    parameters = load_file(file_path)

    # Replace placeholders.
    if placeholders is not None:
        replace_placeholders(parameters[0][0], placeholders)

    # Adapt coordinates.
    is_adapt = False
    source_frame_id = ''
    target_frame_id = ''
    position = [0, 0, 0]
    orientation = [0, 0, 0, 1]
    if 'adapt_coordinates' in parameters[0][0]:
        is_adapt = True
        adapt_parameters = parameters[0][0]['adapt_coordinates'][0]['transform']
        source_frame_id = adapt_parameters['source_frame']
        target_frame_id = adapt_parameters['target_frame']
        if 'transform_in_source_frame' in adapt_parameters:
            if 'position' in adapt_parameters['transform_in_source_frame']:
                position = adapt_parameters['transform_in_source_frame']['position']
            if 'orientation' in adapt_parameters['transform_in_source_frame']:
                orientation = adapt_parameters['transform_in_source_frame']['orientation']
                if len(orientation) == 3:
                    orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2])

    if is_adapt:
        try:
            (position, orientation) = transform_coordinates(source_frame_id, target_frame_id, position, orientation)
        except TypeError:
            return None

    return parse_action(parameters, source_frame_id, target_frame_id, position, orientation)


def load_action_from_file_and_transform(file_path, source_frame_id='', position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    from rosparam import load_file
    import os

    if not os.path.isfile(file_path):
        rospy.logerr('File with path "' + file_path + '" does not exists.')
        return None

    return parse_action(load_file(file_path), source_frame_id, position, orientation)


def parse_action(yaml_object, source_frame_id='', target_frame_id='', position=[0, 0, 0], orientation=[0, 0, 0, 1]):
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
            if 'end_effector_target' in motion_parameter:
                step.end_effector_target.append(parse_end_effector_target(motion_parameter['end_effector_target']))
            if 'end_effector_trajectory' in motion_parameter:
                step.end_effector_trajectory.append(parse_end_effector_trajectory(motion_parameter['end_effector_trajectory']))
            if 'leg_mode' in motion_parameter:
                step.leg_mode.append(parse_leg_mode(motion_parameter['leg_mode']))
            if 'joint_trajectory' in motion_parameter:
                step.joint_trajectory.append(parse_joint_trajectory(motion_parameter['joint_trajectory']))
            if 'base_auto' in motion_parameter:
                step.base_auto.append(parse_base_auto(motion_parameter['base_auto']))
            if 'base_target' in motion_parameter:
                step.base_target.append(parse_base_target(motion_parameter['base_target']))
            if 'base_trajectory' in motion_parameter:
                step.base_trajectory.append(parse_base_trajectory(motion_parameter['base_trajectory']))
            if 'custom_command' in motion_parameter:
                step.custom_command.append(parse_custom_command(motion_parameter['custom_command']))

        goal.steps.append(step)

    # Adapt to local coordinates if desired.
    if not (numpy.array_equal(position, [0, 0, 0]) and numpy.array_equal(orientation, [0, 0, 0, 1])):
        adapt_coordinates(goal, source_frame_id, target_frame_id, position, orientation)

    # print goal
    return goal


def replace_placeholders(yaml_object, placeholders):
    if type(yaml_object) == dict:
        for i, item in yaml_object.items():
            if type(item) == str:
                if item in placeholders:
                    yaml_object[i] = placeholders[item]
            else:
                replace_placeholders(yaml_object[i], placeholders)
    if type(yaml_object) == list:
        for i, item in enumerate(yaml_object):
            if type(item) == str:
                if item in placeholders:
                    yaml_object[i] = placeholders[item]
            else:
                replace_placeholders(yaml_object[i], placeholders)


def parse_footstep(yaml_object):
    footstep = free_gait_msgs.msg.Footstep()
    if not yaml_object:
        return footstep
    if 'name' in yaml_object:
        footstep.name = yaml_object['name']
    if 'target' in yaml_object:
        footstep.target = parse_position_stamped(yaml_object['target'])
    if 'profile_height' in yaml_object:
        footstep.profile_height = yaml_object['profile_height']
    if 'average_velocity' in yaml_object:
        footstep.average_velocity = yaml_object['average_velocity']
    if 'profile_type' in yaml_object:
        footstep.profile_type = yaml_object['profile_type']
    if 'ignore_contact' in yaml_object:
        footstep.ignore_contact = yaml_object['ignore_contact']
    if 'surface_normal' in yaml_object:
        footstep.surface_normal = parse_vector_stamped(yaml_object['surface_normal'])
    if 'ignore_for_pose_adaptation' in yaml_object:
        footstep.ignore_for_pose_adaptation = yaml_object['ignore_for_pose_adaptation']
    return footstep


def parse_end_effector_target(yaml_object):
    end_effector_target = free_gait_msgs.msg.EndEffectorTarget()
    if not yaml_object:
        return end_effector_target
    if 'name' in yaml_object:
        end_effector_target.name = yaml_object['name']
    if 'target_position' in yaml_object:
        end_effector_target.target_position.append(parse_position_stamped(yaml_object['target_position']))
    if 'target_velocity' in yaml_object:
        end_effector_target.target_velocity.append(parse_vector_stamped(yaml_object['target_velocity']))
    if 'target_acceleration' in yaml_object:
        end_effector_target.target_acceleration.append(parse_vector_stamped(yaml_object['target_acceleration']))
    if 'target_force' in yaml_object:
        end_effector_target.target_force.append(parse_vector_stamped(yaml_object['target_force']))
    if 'average_velocity' in yaml_object:
        end_effector_target.average_velocity = yaml_object['average_velocity']
    if 'duration' in yaml_object:
        end_effector_target.duration = yaml_object['duration']
    if 'Kp' in yaml_object:
        end_effector_target.k_p.append(parse_vector_stamped(yaml_object['Kp']))
    if 'Kd' in yaml_object:
        end_effector_target.k_d.append(parse_vector_stamped(yaml_object['Kd']))
    if 'Kf' in yaml_object:
        end_effector_target.k_f.append(parse_vector_stamped(yaml_object['Kf']))        
    if 'ignore_contact' in yaml_object:
        end_effector_target.ignore_contact = yaml_object['ignore_contact']
    if 'surface_normal' in yaml_object:
        end_effector_target.surface_normal = parse_vector_stamped(yaml_object['surface_normal'])
    if 'ignore_for_pose_adaptation' in yaml_object:
        end_effector_target.ignore_for_pose_adaptation = yaml_object['ignore_for_pose_adaptation']
    return end_effector_target


def parse_end_effector_trajectory(yaml_object):
    end_effector_trajectory = free_gait_msgs.msg.EndEffectorTrajectory()
    if not yaml_object:
        return end_effector_trajectory
    if 'name' in yaml_object:
        end_effector_trajectory.name = yaml_object['name']
    if 'trajectory' in yaml_object:
        end_effector_trajectory.trajectory = parse_translational_trajectory(end_effector_trajectory.name, yaml_object['trajectory'])
    if 'Kp' in yaml_object:
        end_effector_trajectory.k_p.append(parse_vector_stamped(yaml_object['Kp']))
    if 'Kd' in yaml_object:
        end_effector_trajectory.k_d.append(parse_vector_stamped(yaml_object['Kd']))
    if 'Kf' in yaml_object:
        end_effector_trajectory.k_f.append(parse_vector_stamped(yaml_object['Kf'])) 
    if 'force_instead_of_acceleration' in yaml_object:
        end_effector_trajectory.force_instead_of_acceleration = yaml_object['force_instead_of_acceleration']
    if 'surface_normal' in yaml_object:
        end_effector_trajectory.surface_normal = parse_vector_stamped(yaml_object['surface_normal'])
    if 'ignore_contact' in yaml_object:
        end_effector_trajectory.ignore_contact = yaml_object['ignore_contact']
    if 'ignore_for_pose_adaptation' in yaml_object:
        end_effector_trajectory.ignore_for_pose_adaptation = yaml_object['ignore_for_pose_adaptation']
    return end_effector_trajectory


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
        leg_mode.surface_normal = parse_vector_stamped(yaml_object['surface_normal'])
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
        joint_trajectory.surface_normal = parse_vector_stamped(yaml_object['surface_normal'])
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


def parse_base_target(yaml_object):
    base_target = free_gait_msgs.msg.BaseTarget()
    if not yaml_object:
        return base_target
    if 'target' in yaml_object:
        base_target.target = parse_pose_stamped(yaml_object['target'])
    if 'ignore_timing_of_leg_motion' in yaml_object:
        base_target.ignore_timing_of_leg_motion = yaml_object['ignore_timing_of_leg_motion']
    if 'average_linear_velocity' in yaml_object:
        base_target.average_linear_velocity = yaml_object['average_linear_velocity']
    if 'average_angular_velocity' in yaml_object:
        base_target.average_angular_velocity = yaml_object['average_angular_velocity']
    return base_target


def parse_base_trajectory(yaml_object):
    base_trajectory = free_gait_msgs.msg.BaseTrajectory()
    if not yaml_object:
        return base_trajectory
    if 'trajectory' in yaml_object:
        base_trajectory.trajectory = parse_multi_dof_trajectory('base', yaml_object['trajectory'])
    return base_trajectory


def parse_custom_command(yaml_object):
    custom_command = free_gait_msgs.msg.CustomCommand()
    if not yaml_object:
        return custom_command
    if 'type' in yaml_object:
        custom_command.type = yaml_object['type']
    if 'duration' in yaml_object:
        custom_command.duration = parse_duration(yaml_object['duration'])
    if 'command' in yaml_object:
        custom_command.command = yaml_object['command']
    return custom_command


def parse_duration(duration):
    return rospy.Duration(duration)


def parse_position(yaml_object):
    point = geometry_msgs.msg.Point()
    point.x = yaml_object[0]
    point.y = yaml_object[1]
    point.z = yaml_object[2]
    return point


def parse_orientation(yaml_object):
    quaternion = geometry_msgs.msg.Quaternion()
    if len(yaml_object) == 4:
        quaternion.x = yaml_object[0]
        quaternion.y = yaml_object[1]
        quaternion.z = yaml_object[2]
        quaternion.w = yaml_object[3]
    elif len(yaml_object) == 3:
        q = quaternion_from_euler(yaml_object[0], yaml_object[1], yaml_object[2])
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
    return quaternion


def parse_vector(yaml_object):
    vector = geometry_msgs.msg.Vector3()
    vector.x = yaml_object[0]
    vector.y = yaml_object[1]
    vector.z = yaml_object[2]
    return vector


def parse_transform(yaml_object):
    transform = geometry_msgs.msg.Transform()
    if 'position' in yaml_object:
        transform.translation = parse_vector(yaml_object['position'])
    if 'orientation' in yaml_object:
        transform.rotation = parse_orientation(yaml_object['orientation'])
    return transform
    
def parse_twist_velocities(yaml_object):
    twist = geometry_msgs.msg.Twist()
    if 'velocity' in yaml_object:
        twist.linear = parse_vector(yaml_object['velocity'])
    return twist
    
def parse_twist_accelerations(yaml_object):
    twist = geometry_msgs.msg.Twist()
    if 'acceleration' in yaml_object:
        twist.linear = parse_vector(yaml_object['acceleration'])
    return twist


def parse_position_stamped(yaml_object):
    point = geometry_msgs.msg.PointStamped()
    if 'frame' in yaml_object:
        point.header.frame_id = yaml_object['frame']
    if 'position' in yaml_object:
        point.point = parse_position(yaml_object['position'])
    return point


def parse_pose_stamped(yaml_object):
    pose = geometry_msgs.msg.PoseStamped()
    if 'frame' in yaml_object:
        pose.header.frame_id = yaml_object['frame']
    if 'position' in yaml_object:
        pose.pose.position = parse_position(yaml_object['position'])
    if 'orientation' in yaml_object:
        pose.pose.orientation = parse_orientation(yaml_object['orientation'])
    return pose


def parse_vector_stamped(yaml_object):
    vector = geometry_msgs.msg.Vector3Stamped()
    if 'frame' in yaml_object:
        vector.header.frame_id = yaml_object['frame']
    if 'vector' in yaml_object:
        vector.vector = parse_vector(yaml_object['vector'])
    return vector


def parse_multi_dof_trajectory(joint_name, trajectory):
    output = trajectory_msgs.msg.MultiDOFJointTrajectory()
    if 'frame' in trajectory:
        output.header.frame_id = trajectory['frame']
    output.joint_names.append(joint_name)
    for knot in trajectory['knots']:
        point = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
        point.time_from_start = rospy.Time(knot['time'])
        transform = parse_transform(knot)
        point.transforms.append(transform)
        output.points.append(point)
    return output


def parse_translational_trajectory(joint_name, trajectory):
    output = trajectory_msgs.msg.MultiDOFJointTrajectory()
    output.header.frame_id = trajectory['frame']
    output.joint_names.append(joint_name)
    for knot in trajectory['knots']:
        point = trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
        point.time_from_start = rospy.Time(knot['time'])
        if 'position' in knot or 'orientation' in knot:
          transform = parse_transform(knot)
          point.transforms.append(transform)
        if 'velocity' in knot:
          twist_velocity = parse_twist_velocities(knot)
          point.velocities.append(twist_velocity)
        if 'acceleration' in knot:
          twist_acceleration = parse_twist_accelerations(knot)
          point.accelerations.append(twist_acceleration)
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


def adapt_coordinates(goal, source_frame_id, target_frame_id, position, orientation):
    # For each step.
    translation = translation_matrix(position)
    yaw = 0
    if len(orientation) == 4:
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
    elif len(orientation) == 3:
        yaw = orientation[2]
    z_axis = [0, 0, 1]
    rotation = rotation_matrix(yaw, z_axis)
    transform = concatenate_matrices(translation, rotation)
    adapt_coordinates_recursively(goal.steps, source_frame_id, target_frame_id, transform)


def adapt_coordinates_recursively(message, source_frame_id, target_frame_id, transform):

    # Stop recursion for methods and primitive types.
    if hasattr(message, '__call__') or isinstance(message, int) or isinstance(message, str) or \
            isinstance(message, bool) or isinstance(message, float):
        return

    # Transform known geometries.
    if isinstance(message, geometry_msgs.msg.Vector3Stamped):
        if check_if_vector_valid(message.vector) and message.header.frame_id == source_frame_id:
            message.header.frame_id = target_frame_id
            vector = transform_vector(transform, message.vector)
            message.vector = vector
        return
    elif isinstance(message, geometry_msgs.msg.PointStamped):
        if check_if_position_valid(message.point) and message.header.frame_id == source_frame_id:
            message.header.frame_id = target_frame_id
            position = transform_position(transform, message.point)
            message.point = position
        return
    elif isinstance(message, geometry_msgs.msg.PoseStamped):
        if check_if_pose_valid(message.pose) and message.header.frame_id == source_frame_id:
            message.header.frame_id = target_frame_id
            pose = transform_pose(transform, message.pose)
            message.pose = pose
        return
    elif isinstance(message, trajectory_msgs.msg.MultiDOFJointTrajectory):
        if message.header.frame_id == source_frame_id:
            message.header.frame_id = target_frame_id
            for i, point in enumerate(message.points):
                for j, transformation in enumerate(message.points[i].transforms):
                    t = transform_transformation(transform, transformation)
                    message.points[i].transforms[j] = t
        return

    # Do recursion for lists and members.
    if hasattr(message, '__iter__'):
        for m in message:  # TODO Need enumerate?
            adapt_coordinates_recursively(m, source_frame_id, target_frame_id, transform)
    else:
        for m in [a for a in dir(message) if not (a.startswith('__') or a.startswith('_') or \
                a == 'deserialize' or a == 'deserialize_numpy' or a == 'serialize' or a == 'serialize_numpy')]:
            adapt_coordinates_recursively(eval("message." + m), source_frame_id, target_frame_id, transform)

# Position and orientation defined in source frame.
def transform_coordinates(source_frame_id, target_frame_id, position = [0, 0, 0], orientation = [0, 0, 0, 1], tf_buffer = None):

    try:
        (translation, rotation) = get_tf_transform(source_frame_id, target_frame_id, tf_buffer)
    except TypeError:
        return None

    transformed_position = translation + quaternion_matrix(rotation)[:3, :3].dot(position)
    transformed_orientation = quaternion_multiply(rotation, orientation)
    return transformed_position, transformed_orientation


def get_transform(source_frame_id, target_frame_id, tf_buffer = None):

    (translation, rotation) = get_tf_transform(source_frame_id, target_frame_id, tf_buffer)
    translation_matrix_form = translation_matrix(translation)
    rotation_matrix_form = quaternion_matrix(rotation)
    return concatenate_matrices(translation_matrix_form, rotation_matrix_form)


def get_tf_transform(source_frame_id, target_frame_id, tf_buffer = None):

    listener = None
    if tf_buffer is None:
        tf_buffer = tf2_ros.Buffer()
        listener = LocalTransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform(target_frame_id, source_frame_id, rospy.Time(), rospy.Duration(10.0))
        if listener is not None:
            listener.unregister()
            del listener
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr('Could not look up TF transformation from "' +
                     source_frame_id + '" to "' + target_frame_id + '".')
        return None

    t = transform.transform.translation
    r = transform.transform.rotation
    return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]


def transform_vector(transform, vector):
    angle, direction, point = rotation_from_matrix(transform)
    transformed_vector = rotation_matrix(angle, direction).dot([vector.x, vector.y, vector.z, 1.0])
    return geometry_msgs.msg.Vector3(transformed_vector[0], transformed_vector[1], transformed_vector[2])


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


def check_if_vector_valid(vector):
    if vector.x == 0 and vector.y == 0 and vector.z == 0:
        return False
    else:
        return True


def check_if_position_valid(position):
    if position.x == 0 and position.y == 0 and position.z == 0:
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


# TODO: We are using this from a newer version. Remove once updated.
class LocalTransformListener():

    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """
    def __init__(self, buffer):
        """
        .. function:: __init__(buffer)

            Constructor.

            :param buffer: The buffer to propagate changes to when tf info updates.
        """
        self.buffer = buffer
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.static_callback)

    def __del__(self):
        self.unregister()

    def unregister(self):
        """
        Unregisters all tf subscribers.
        """
        self.tf_sub.unregister()
        self.tf_static_sub.unregister()

    def callback(self, data):
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
