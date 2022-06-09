from koco_action.exceptions import UnparsableGoal, EmptyGoal
from koco_conversions.pose_conversions import convert_pose_to_array_7 as pose_to_array
import tf


def movej_joints_to_array(goal):
    try:
        if goal.target_joint.position:
            return goal.target_joint.position
        if goal.target_array:
            return goal.target_array
    except Exception as e:
        raise UnparsableGoal("Incorrect input to parser!\nInput: {}".format(goal))

    raise EmptyGoal("None of the goal fields were specified!")


def movej_pose_to_array(goal):
    try:
        return pose_to_array(goal.target_pose)
    except ArithmeticError:
        raise EmptyGoal("None of the goal fields were specified!")
    except Exception as e:
        raise UnparsableGoal("Parser could not handle the goal.\nReason:{0}\nGoal:{1}".format(e, goal))


def movel_goal_to_array(goal):
    return [
        goal.target_pose.position.x,
        goal.target_pose.position.y,
        goal.target_pose.position.z,
        goal.target_pose.orientation.x,
        goal.target_pose.orientation.y,
        goal.target_pose.orientation.z,
        goal.target_pose.orientation.w]


def quaternion_to_rpy(quaternion):
    return list(tf.transformations.euler_from_quaternion(quaternion))


def posequat_to_poserpy(pose_array):
    return pose_array[0:2] + quaternion_to_rpy(pose_array[3:6])
