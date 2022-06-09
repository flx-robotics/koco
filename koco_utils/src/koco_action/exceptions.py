# Specific exceptions
class RobotNotReady(Exception):
    """Robot is not ready to execute the movement"""


class MovementNotStarted(Exception):
    """Command sent but movement did not start"""


class RobotInError(Exception):
    """The robot is in error state"""


class UnparsableGoal(Exception):
    """Cannot parse action goal"""


class EmptyGoal(Exception):
    """The goal message is empty"""


class ActionPreempted(Exception):
    """Action was preempted"""


class GoalTooClose(Exception):
    """The provided goal is too close to the robot's current pose"""
