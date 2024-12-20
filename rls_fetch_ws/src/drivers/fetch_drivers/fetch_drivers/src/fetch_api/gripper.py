#! /usr/bin/env python

import actionlib
import control_msgs.msg
import rospy

ACTION_SERVER = 'gripper_controller/gripper_action'

# ------ Constants ----------
GRIPPER_OPENING_OFFSET = -0.004 # for some reason, there is a constant offset here

class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons
    CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
    OPENED_POS = 0.10  # The position for a fully-open gripper (meters).

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = Gripper.OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, opening=CLOSED_POS, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = opening + GRIPPER_OPENING_OFFSET
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

