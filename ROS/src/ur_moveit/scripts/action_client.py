#!/usr/bin/env python
from __future__ import print_function

import rospy

import time
import sys
import copy
import math
import moveit_commander
import actionlib

import moveit_msgs.msg

from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,FollowJointTrajectoryActionGoal #JointTrajectoryAction,JointTrajectoryGoal,JointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def plan_single_trajectory(req):
    print("Starting planning....")
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    print("Set up action server")

    #setup_scene()
    #group_name = "arm"
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #move_group.set_planner_id("RRTstar")

    # Add table collider to MoveIt scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0, -0.02])#+0.77])
    scene.add_box("table", p, (1.5, 1.5, 0.01))
    p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0.4, 0])
    scene.add_box("back_wall", p, (0.5, 0.1, 1))
    #print(f"collision objects: {scene.get_objects()}")
    print("Set up scene")

    #response = MoverServiceResponse()

    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

    plan = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    #previous_ending_joint_angles = target_pose.joint_trajectory.points[-1].positions
    #response.trajectories.append(target_pose)

    move_group.clear_pose_targets()

    joint_trajectory = FollowJointTrajectoryGoal()
    joint_trajectory.goal_time_tolerance = rospy.Time(0.1)
    joint_trajectory.trajectory = plan.joint_trajectory
    #joint_trajectory.trajectory.points = plan.joint_trajectory.points
    #joint_trajectory.trajectory.header.stamp = rospy.Time.now()

    #return response
    #goal = FollowJointTrajectoryGoal()
    client.send_goal(joint_trajectory)
    #client.wait_for_result()
    print("Sent goal")
    


"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles):
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, start_joint_angles)
        raise Exception(exception_str)
    return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.

    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position
    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved
    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    setup_scene()

    response = MoverServiceResponse()

    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_pose)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.075  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_pose)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)

    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_pose)

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(place_pose)

    # Post place - lift gripper after placement position
    place_pose = copy.deepcopy(req.place_pose)
    place_pose.position.z += 0.075 # Static value offset to lift up gripper
    post_place_pose = plan_trajectory(move_group, place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = post_place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(post_place_pose)

    move_group.clear_pose_targets()
    return response

def setup_scene():
    global move_group

    #group_name = "arm"
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Add table collider to MoveIt scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0, -0.02])#+0.77])
    scene.add_box("table", p, (20, 20, 0.01))
    print(f"collision objects: {scene.getKnownCollisionObjects()}")
    
    

def set_pose(poseStamped, pose):
    '''
    pose is an array: [x, y, z]
    '''
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    return poseStamped
def on_goal(goal_handle):
    goal_handle.get_goal().trajectory
    return
def on_cancel(goal_handle):
    goal_handle.get_goal().trajectory
    return


def moveit_server():
    #moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('ur_moveit_server')

    #s = rospy.Service('ur_moveit', MoverService, plan_single_trajectory)
    #print("Ready to plan")

    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur_action_clients')#,anonymous=True)

    s = rospy.Subscriber('ur_action_client', MoverServiceRequest, plan_single_trajectory)

    print("Action client ready.")

    rospy.spin()

#def other_server():
#    rospy.init_node('driver', anonymous=True, disable_signals=True)
    
#    global server
#    server = actionlib.ActionServer("scaled_pos_joint_traj_controller/follow_joint_trajectory",
#                                             FollowJointTrajectoryAction,
#                                             on_goal, on_cancel, auto_start=False)
#    server.start()
#    rospy.spin()

if __name__ == "__main__":
    moveit_server()