#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib

import moveit_msgs.msg

from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,FollowJointTrajectoryActionGoal #JointTrajectoryAction,JointTrajectoryGoal,JointTrajectoryActionGoal

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def on_goal(goal_handle):
    goal_handle.get_goal().trajectory
    return
def on_cancel(goal_handle):
    goal_handle.get_goal().trajectory
    return


def other_server():
    rospy.init_node('ur_action_server', anonymous=True, disable_signals=True)
    
    global server
    server = actionlib.ActionServer("scaled_pos_joint_traj_controller/follow_joint_trajectory",
                                            FollowJointTrajectoryAction,
                                            on_goal, on_cancel, auto_start=False)
    server.start()
    print("Action server ready.")
    rospy.spin()

if __name__ == "__main__":
    other_server()
