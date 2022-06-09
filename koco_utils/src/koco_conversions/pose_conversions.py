import copy
from math import radians
import numpy as np
from numpy.linalg import norm
import rospy
import tf
import tf.transformations as tfs
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Point, Quaternion, Vector3


def offset_to_pose(pose, offset):
    """Adds an offset to the pose frame.
    """

    try:
        # Offset is as array of 6 (tx, ty, tz, rx, ry, rz)
        offset_pose = convert_array_6_to_pose(offset)
    except Exception as e:
        print('Failed to parse as array of 6')
        print("Exception:\n{0}".format(e))
        try:
            # Offset is as array of 7 (tx, ty, tz, rx, ry, rz, rw)
            offset_pose = convert_array_7_to_pose(offset)
        except:
            print('Failed to parse as array of 7')
            print("Exception:\n{0}".format(e))
            try:
                # Offset is as Pose()
                if not isinstance(offset, Pose):
                    raise Exception()
                offset_pose = offset
            except Exception as e:
                print("Could not parse pose !!")
                print("Exception:\n{0}".format(e))
                return False

    offset_transform = tf2_geometry_msgs.do_transform_pose(
        PoseStamped(pose=offset_pose),
        TransformStamped(transform=convert_pose_to_transform(pose))
    )

    return offset_transform


def offset_to_transform(transform, offset):
    """Adds an offset to the transform frame
    """

    try:
        # Offset is as array of 6 (tx, ty, tz, rx, ry, rz)
        offset_pose = convert_array_6_to_pose(offset)
    except Exception as e:
        print('Failed to parse as array of 6')
        print("Exception:\n{0}".format(e))
        try:
            # Offset is as array of 7 (tx, ty, tz, rx, ry, rz, rw)
            offset_pose = convert_array_7_to_pose(offset)
        except:
            print('Failed to parse as array of 7')
            print("Exception:\n{0}".format(e))
            try:
                # Offset is as Pose()
                if not isinstance(offset, Pose):
                    raise Exception()
                offset_pose = offset
            except Exception as e:
                print("Could not parse pose !!")
                print("Exception:\n{0}".format(e))
                return False
    offset_pose = tf2_geometry_msgs.do_transform_pose(
        PoseStamped(pose=offset_pose),
        TransformStamped(transform=transform)).pose

    return convert_pose_to_transform(offset_pose)


def offset_to_wobj(offset, wobj, pose):
    offset_transform = offset_to_pose(wobj, offset)
    offset_target = Pose()
    offset_target.position.x = pose.position.x + \
        offset_transform.pose.position.x - wobj.position.x
    offset_target.position.y = pose.position.y + \
        offset_transform.pose.position.y - wobj.position.y
    offset_target.position.z = pose.position.z + \
        offset_transform.pose.position.z - wobj.position.z
    offset_target.orientation.x = pose.orientation.x
    offset_target.orientation.y = pose.orientation.y
    offset_target.orientation.z = pose.orientation.z
    offset_target.orientation.w = pose.orientation.w
    return offset_target


def convert_array_7_to_pose(array):
    position = Point(x=array[0], y=array[1], z=array[2])
    orientation = Quaternion(x=array[3], y=array[4], z=array[5], w=array[6])
    pose = Pose(position=position, orientation=orientation)

    return pose


def convert_array_6_to_pose(array):
    q_rot = tf.transformations.quaternion_from_euler(
        array[3], array[4], array[5])
    position = Point(x=array[0], y=array[1], z=array[2])
    orientation = Quaternion(x=q_rot[0], y=q_rot[1], z=q_rot[2], w=q_rot[3])
    pose = Pose(position=position, orientation=orientation)

    return pose


def convert_pose_to_array_6(pose):
    orientation = [pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w]
    norm_cond = abs(norm(orientation) - 1.0)
    if norm_cond > 0.0001:
        raise ValueError("Quaternion norm not 1. Diff with actual: {:.15f}".format(norm_cond))
    position = [pose.position.x, pose.position.y, pose.position.z]
    euler = tf.transformations.euler_from_quaternion(orientation)

    return position + list(euler)


def convert_pose_to_array_7(pose):
    orientation = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w]
    ori_norm = norm(orientation)
    if ori_norm == 0.0:
        raise ArithmeticError("This pose was not specified properly!")
    if not(ori_norm == 0.0):
        position = [
            pose.position.x,
            pose.position.y,
            pose.position.z]
    return position + orientation


def convert_2d_list_to_pose(list):
    position = Point(x=list[0][0], y=list[0][1], z=list[0][2])
    orientation = Quaternion(
        x=list[1][0], y=list[1][1], z=list[1][2], w=list[1][3])
    pose = Pose(position=position, orientation=orientation)

    return pose


def convert_pose_to_transform(pose):
    translation = Vector3(
        x=pose.position.x, y=pose.position.y, z=pose.position.z)
    rotation = pose.orientation
    transform = Transform(translation=translation, rotation=rotation)

    return transform


def convert_transform_to_pose(transform):

    position = Point(
        x=transform.translation.x,
        y=transform.translation.y,
        z=transform.translation.z)
    orientation = transform.rotation
    pose = Pose(position=position, orientation=orientation)

    return pose


def convert_transform_to_matrix(transform):
    t_mat = tf.transformations.translation_matrix(
        (transform.translation.x, transform.translation.y, transform.translation.z))
    t_rot = tf.transformations.quaternion_matrix(
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
    matrix = tf.transformations.concatenate_matrices(t_mat, t_rot)
    return matrix


def convert_matrix_to_transform(matrix):
    translation = tf.transformations.translation_from_matrix(matrix)
    quat = tf.transformations.quaternion_from_matrix(matrix)
    transform = Transform(translation=Vector3(
        translation[0], translation[1], translation[2]), rotation=Quaternion(quat[0], quat[1], quat[2], quat[3]))
    return transform


def quat_obj_to_quat_array(quat):
    return [
        quat.x,
        quat.y,
        quat.z,
        quat.w]


def quat_array_to_quat_obj(quat):
    return Quaternion(
        x=quat[0],
        y=quat[1],
        z=quat[2],
        w=quat[3])


def array_6_to_matrix(array, rotation_format='sxyz', degrees=False):
    """
    Converts a pose array to matrix form.
    Args:
        array(list): Array with format: [x, y, z, rx, ry, rz].
        rotation_format(str): Format of the encoded rotation.
            'rodrigues' - Interprets rotation encoded in the angle-axis format.
            Any of the 24 axis sequences supported by tf.transformations.euler_matrix() - interprets rotation as Euler.
        degrees(bool): If True, interprets the rx, ry, and rz in degrees. Default is radians.

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    if degrees:
        array = [array[0], array[1], array[2], radians(array[3]), radians(array[4]), radians(array[5])]

    if rotation_format == 'rodrigues':
        angle = np.linalg.norm(array[3:])
        axis = np.array(array[3:])/angle
        mat = tfs.rotation_matrix(angle, axis)
    else:
        mat = tfs.euler_matrix(array[3], array[4], array[5], rotation_format)

    mat[:3, 3] = array[:3]
    return mat


def array_7_to_matrix(array):
    """
    Converts a pose array to matrix form. Rotations are in unit quaternions.
    Args:
        array(list): Array with format: [x, y, z, qx, qy, qz, qw].

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    mat = tfs.quaternion_matrix(array[3:])
    mat[:3, 3] = array[:3]
    return mat


def matrix_to_quaternion(transformation_matrix):
    """
    Extracts a quaternion array from a homogeneous transformation matrix.
    Args:
        transformation_matrix(np.ndarray): Homogeneous transformation matrix.

    Returns:
        np.ndarray: Quaternion array - [x, y, z, w].
    """
    return tfs.quaternion_from_matrix(transformation_matrix)


def matrix_to_quaternion_msg(transformation_matrix):
    """
    Composes a Quaternion message from a homogeneous transformation matrix.
    Args:
        transformation_matrix(np.ndarray): Homogeneous transformation matrix.

    Returns:
        Quaternion: Quaternion message.
    """
    q = matrix_to_quaternion(transformation_matrix)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def matrix_to_transform(transformation_matrix):
    """
    Composes a Transform message from a homogeneous transformation matrix.
    Args:
        transformation_matrix(np.ndarray): Homogeneous transformation matrix.

    Returns:
        Transform: Transform message.
    """
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = transformation_matrix[:3, 3]
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = matrix_to_quaternion(transformation_matrix)
    return t


def matrix_to_transform_stamped(transformation_matrix, frame_id, child_frame_id):
    """
    Composes a TransformStamped message from a homogeneous transformation matrix.
    Args:
        transformation_matrix(np.ndarray): Homogeneous transformation matrix.
        frame_id(str): Parent frame ID.
        child_frame_id(str): Child frame ID.

    Returns:
        TransformStamped: TransformStamped message.
    """
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform = matrix_to_transform(transformation_matrix)
    return t


def matrix_to_pose(transformation_matrix):
    """
    Composes pose message from transform array.
    Args:
        transformation_matrix(np.ndarray): Transformation matrix.

    Returns:
        Pose: Pose message.
    """
    p = Pose()
    p.position.x, p.position.y, p.position.z = transformation_matrix[:3, 3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = matrix_to_quaternion(transformation_matrix)
    return p


def matrix_to_pose_stamped(transformation_matrix, frame_id):
    """
    Composes pose message from transform array.
    Args:
        transformation_matrix(np.array): Transformation matrix.
        frame_id(str): Parent frame ID.

    Returns:
        PoseStamped: PoseStamped message.
    """
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose = matrix_to_pose(transformation_matrix)
    return p


def transform_to_translation(transform_msg):
    """
    Extracts translation vector from a Transform message.
    Args:
        transform_msg(Transform): Transform message.

    Returns:
        np.ndarray: Translation vector - [x, y, z].
    """
    return np.array([transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z])


def transform_to_quaternion(transform_msg):
    """
    Extracts quaternion vector from a Transform message.
    Args:
        transform_msg(Transform): Transform message.

    Returns:
        np.ndarray: Quaternion vector - [x, y, z, w].
    """
    return np.array([transform_msg.rotation.x, transform_msg.rotation.y,
                     transform_msg.rotation.z, transform_msg.rotation.w])


def transform_to_matrix(transform_msg):
    """
    Extracts transformation matrix from a Transform message.
    Args:
        transform_msg(Transform): Transform message.

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    return np.dot(tfs.translation_matrix(transform_to_translation(transform_msg)),
                  tfs.quaternion_matrix(transform_to_quaternion(transform_msg)))


def transform_to_transform_stamped(transform_msg, frame_id, child_frame_id):
    """
    Composes a TransformStamped message from a homogeneous transformation matrix.
    Args:
        transform_msg(Transform): Transform message.
        frame_id(str): Parent frame ID.
        child_frame_id(str): Child frame ID.

    Returns:
        TransformStamped: TransformStamped message.
    """
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform = copy.deepcopy(transform_msg)
    return t


def transform_to_pose(transform_msg):
    """
    Converts transform message to pose message.
    Args:
        transform_msg(Transform): Transform message.

    Returns:
        Pose: Pose message.
    """
    p = Pose()
    p.position.x = transform_msg.translation.x
    p.position.y = transform_msg.translation.y
    p.position.z = transform_msg.translation.z
    p.orientation = copy.deepcopy(transform_msg.rotation)
    return p


def transform_to_pose_stamped(transform_msg, frame_id):
    """
    Converts transform message to pose message.
    Args:
        transform_msg(Transform): Transform message.
        frame_id(str): Parent frame ID.

    Returns:
        PoseStamped: PoseStamped message.
    """
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose = transform_to_pose(transform_msg)
    return p


def transform_stamped_to_translation(transform_msg):
    """
    Extracts translation vector from a TransformStamped message.
    Args:
        transform_msg(TransformStamped): Transform message.

    Returns:
        np.ndarray: Translation vector - [x, y, z].
    """
    return transform_to_translation(transform_msg.transform)


def transform_stamped_to_quaternion(transform_msg):
    """
    Extracts quaternion vector from a TransformStamped message.
    Args:
        transform_msg(TransformStamped): Transform message.

    Returns:
        np.ndarray: Quaternion vector - [x, y, z, w].
    """
    return transform_to_quaternion(transform_msg.transform)


def transform_stamped_to_matrix(transform_msg):
    """
    Extracts transformation matrix from a TransformStamped message.
    Args:
        transform_msg(TransformStamped): Transform message.

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    return transform_to_matrix(transform_msg.transform)


def transform_stamped_to_pose(transform_msg):
    """
    Converts transform message to pose message.
    Args:
        transform_msg(TransformStamped): Transform message.

    Returns:
        Pose: Pose message.
    """
    return transform_to_pose(transform_msg.transform)


def transform_stamped_to_pose_stamped(transform_msg):
    """
    Converts transform message to pose message.
    Args:
        transform_msg(TransformStamped): Transform message.

    Returns:
        PoseStamped: PoseStamped message.
    """
    p = PoseStamped()
    p.header.frame_id = transform_msg.header.frame_id
    p.header.stamp = rospy.Time.now()
    p.pose = transform_to_pose(transform_msg.transform)
    return p


def pose_to_quaternion(pose_msg):
    """
    Extracts quaternion vector from pose message.
    Args:
        pose_msg(Pose): Pose message.

    Returns:
        np.ndarray: Quaternion vector - [x, y, z, w].
    """
    return np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])


def pose_to_translation(pose_msg):
    """
    Extracts translation vector from pose message.
    Args:
        pose_msg(Pose): Pose message.

    Returns:
        np.ndarray: Translation vector - [x, y, z].
    """
    return np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])


def pose_to_matrix(pose_msg):
    """
    Extracts transformation matrix from pose message.
    Args:
        pose_msg(Pose): Pose message.

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    return np.dot(tfs.translation_matrix(pose_to_translation(pose_msg)),
                  tfs.quaternion_matrix(pose_to_quaternion(pose_msg)))


def pose_to_transform(pose_msg):
    """
    Converts pose message to transform message.
    Args:
        pose_msg(Pose): Pose message.

    Returns:
        Transform: Transform message.
    """
    t = Transform()
    t.translation.x = pose_msg.position.x
    t.translation.y = pose_msg.position.y
    t.translation.z = pose_msg.position.z
    t.rotation = copy.deepcopy(pose_msg.orientation)
    return t


def pose_to_transform_stamped(pose_msg, frame_id, child_frame_id):
    """
    Converts pose message to transform message.
    Args:
        pose_msg(Pose): Pose message.
        frame_id(str): Parent frame ID.
        child_frame_id(str): Child frame ID.

    Returns:
        TransformStamped: TransformStamped message.
    """
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform = pose_to_transform(pose_msg)
    return t


def pose_to_pose_stamped(pose_msg, frame_id):
    """
    Converts pose message to pose message.
    Args:
        pose_msg(Pose): Pose message.
        frame_id(str): Parent frame ID.

    Returns:
        PoseStamped: PoseStamped message.
    """
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose = copy.deepcopy(pose_msg)
    return p


def pose_stamped_to_quaternion(pose_msg):
    """
    Extracts quaternion vector from pose message.
    Args:
        pose_msg(PoseStamped): Pose message.

    Returns:
        np.ndarray: Quaternion vector - [x, y, z, w].
    """
    return pose_to_quaternion(pose_msg.pose)


def pose_stamped_to_translation(pose_msg):
    """
    Extracts translation vector from pose message.
    Args:
        pose_msg(PoseStamped): Pose message.

    Returns:
        np.ndarray: Translation vector - [x, y, z].
    """
    return pose_to_translation(pose_msg.pose)


def pose_stamped_to_matrix(pose_msg):
    """
    Extracts transformation matrix from pose message.
    Args:
        pose_msg(PoseStamped): Pose message.

    Returns:
        np.ndarray: Transformation matrix [4,4].
    """
    return pose_to_matrix(pose_msg.pose)


def pose_stamped_to_transform(pose_msg):
    """
    Converts pose message to transform message.
    Args:
        pose_msg(PoseStamped): Pose message.

    Returns:
        Transform: Transform message.
    """
    return pose_to_transform(pose_msg.pose)


def pose_stamped_to_transform_stamped(pose_msg, child_frame_id):
    """
    Converts pose message to transform message.
    Args:
        pose_msg(PoseStamped): Pose message.
        child_frame_id(str): Child frame ID.

    Returns:
        TransformStamped: TransformStamped message.
    """
    t = TransformStamped()
    t.header.frame_id = pose_msg.header.frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform = pose_to_transform(pose_msg.pose)
    return t
