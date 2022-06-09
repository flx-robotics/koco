#!/usr/bin/env python

import rospy
from mongodb_store.message_store import MessageStoreProxy
from koco_msgs.srv import PlanAndStore, PlanAndStoreResponse
import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory
from moveit_msgs.srv import GetStateValidity


GROUPS_PARAM = '/robot_description_kinematics'
MOVEIT_SERVERS_TIMEOUT = 60.0
DEFAULT_PLANNER = ''
DEFAULT_PLANNING_TIME = 5
DEFAULT_ATTEMPTS = 1
DEFAULT_JIGGLE = 0.0
DEFAULT_JIGGLE_STEP = 0.0

# This will stay hardcoded for now. This will have to be adjusted in the future when we want to support multiple robots
ROBOT_NAMESPACE = '/robot_arm'


class PlayandStoreService(object):

    _plan_and_store_srv = 'plan_and_store'
    _group_name = None
    _move_groups = {}

    def __init__(self):

        rospy.init_node('play_and_store_node')
        self.msg_store = MessageStoreProxy()

        rospy.loginfo("Plan and Store service started. Will now wait for MoveIT to come online ...")

        try:
            self._init_move_groups()
        except Exception as e:
            rospy.logerr(
                "Could not connect to MoveIT action servers with the predefined time [{}s]".format(MOVEIT_SERVERS_TIMEOUT))
            rospy.logerr("Exception: {}".format(e))
            exit()
        rospy.loginfo("MoveIT is online and the service is ready to be used.")

        self._init_params()

        self.playstoreservice = rospy.Service(
            self._plan_and_store_srv,
            PlanAndStore,
            self.plan_and_store_cb)

        self._collision_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        try:
            self._collision_srv.wait_for_service(5)
        except rospy.ROSException:
            rospy.logerr("Could not connect to the collision checking service. Exiting ...")
            exit()

        rospy.spin()

    def _init_params(self):
        self._planner_id = rospy.get_param(
            '~planner_id', default=DEFAULT_PLANNER)
        self._planning_time = rospy.get_param(
            '~planning_time', default=DEFAULT_PLANNING_TIME)
        self._n_attempts = rospy.get_param(
            '~attempts', default=DEFAULT_ATTEMPTS)
        self._initial_jiggle = rospy.get_param(
            '~initial_jiggle', default=DEFAULT_JIGGLE)
        self._jiggle_step = rospy.get_param(
            '~jiggle_step', default=DEFAULT_JIGGLE_STEP)

    def _init_move_groups(self):
        move_groups_param = rospy.get_param(GROUPS_PARAM)
        for group in move_groups_param:
            self._move_groups[group] = moveit_commander.MoveGroupCommander(
                group, wait_for_servers=MOVEIT_SERVERS_TIMEOUT)

    def plan_and_store_cb(self, req):
        db_entry_name = req.trajectory_name
        joint_goal_position = req.joint_target
        group_name = req.group_name

        self._move_groups[group_name].clear_path_constraints()
        self._move_groups[group_name].set_planner_id(self._planner_id)
        self._move_groups[group_name].set_start_state_to_current_state()
        try:
            if self._initial_jiggle:
                plan = self.calculate_trajectory_constrained(joint_goal_position, group_name)
            else:
                plan = self.calculate_trajectory_unconstrained(joint_goal_position, group_name)

            if not plan.joint_trajectory.points:
                if self.is_start_in_collision(group_name):
                    return PlanAndStoreResponse(generated_trajectory=plan.joint_trajectory, success=False, messages="Planning was unsuccessful - start state in collision")
                else:
                    return PlanAndStoreResponse(generated_trajectory=plan.joint_trajectory, success=False, messages="Planning was unsuccessful - no solution found")
            if not db_entry_name:
                return PlanAndStoreResponse(generated_trajectory=plan.joint_trajectory, success=True, messages="Trajectory name not given and not stored to DB")
            self.msg_store.update_named(db_entry_name, plan.joint_trajectory, upsert=True)
            return PlanAndStoreResponse(generated_trajectory=plan.joint_trajectory, success=True, messages="Trajectory Succesfully stored")
        except Exception as e:
            rospy.logerr("The Plan and Store service failed due to an exception:\n{}".format(e))
            return PlanAndStoreResponse(success=False, messages=str(e))

    def calculate_trajectory_unconstrained(self, target_joints, group_name):
        return self._move_groups[group_name].plan(target_joints)

    def calculate_trajectory_constrained(self, target_joints, group_name):
        plan = RobotTrajectory()
        try:
            self._move_groups[group_name].set_planning_time(self._planning_time)

            current_joints = self._move_groups[group_name].get_current_joint_values()
            joint_names = self._move_groups[group_name].get_joints()

            rospy.logdebug("Planning from:{0} to {1}".format(
                current_joints, target_joints))

            jiggle = self._initial_jiggle
            for attempt_nr in range(self._n_attempts):
                path_constraints = self.create_joint_constraints(
                    start=current_joints,
                    goal=target_joints,
                    names=joint_names,
                    jiggle=jiggle)
                self._move_groups[group_name].set_path_constraints(
                    path_constraints)

                plan = self._move_groups[group_name].plan(target_joints)

                if plan.joint_trajectory.points:
                    rospy.logdebug("Solution found!")
                    break
                else:
                    rospy.logdebug("Plan not found, increasing the jiggle factor ...")
                    jiggle += self._jiggle_step

            rospy.loginfo("Stopped looking for solution after {0} (out of the allowed {1}) attempts.".format(
                attempt_nr + 1, self._n_attempts))

        except Exception as e:
            rospy.logerr("Failed to compute the plan due to an exception:\n{}".format(e))
        return plan

    def is_start_in_collision(self, group_name):
        robot_state = self._move_groups[group_name].get_current_state()

        collision_result = self._collision_srv(robot_state, group_name, None)
        if collision_result.valid:
            return False
        else:
            rospy.logwarn("Start state is in collision!")
            return True

    @staticmethod
    def create_joint_constraints(start, goal, names=None, jiggle=0.05):
        if names is None:
            try:
                # Either all the joints are provided as JointState or none is
                start_joints = start.position
                goal_joints = goal.position
                joint_names = start.name
            except Exception:
                raise Exception("Joint names must be provided")
        else:
            start_joints = start
            goal_joints = goal
            joint_names = names

        joint_constraints = []
        # Generate joint constraints so they are within the difference of the target and current state (plus jiggle)
        for start_value, end_value, joint_name in zip(start_joints, goal_joints, joint_names):
            joint_diff = end_value - start_value
            lower_bound = abs(joint_diff) + jiggle if joint_diff > 0 else jiggle
            upper_bound = abs(joint_diff) + jiggle if joint_diff < 0 else jiggle
            joint_constraints.append(JointConstraint(
                joint_name, end_value, upper_bound, lower_bound, 1))

        return Constraints(joint_constraints=joint_constraints)


if __name__ == '__main__':

    PlayandStoreService()
    print ('Plan and store service exited cleanly')
