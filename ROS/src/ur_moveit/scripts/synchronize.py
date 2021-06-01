#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import math

from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,FollowJointTrajectoryActionGoal #JointTrajectoryAction,JointTrajectoryGoal,JointTrajectoryActionGoal
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64,Float64MultiArray
from moveit_commander.conversions import pose_to_list

from ur_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse
from std_srvs.srv import Trigger
synchronize_unity = True
if len(sys.argv)>1:
	synchronize_unity = sys.argv[1].lower() == 'true'

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

if(synchronize_unity):
	#rostopic echo -c /joint_states/position
	SUB_NAME = '/joint_states'
	PUB_NAME = '/ur_unity_joints_sub'
	PUB_DATA_TYPE = Float64MultiArray
else:
	SUB_NAME = '/ur_unity_joints_pub'
	PUB_NAME = '/ur_hardware_interface/script_command'
	PUB_DATA_TYPE = String
p = rospy.Publisher(PUB_NAME,PUB_DATA_TYPE, queue_size=10)


def synchronize_robots(req):
	joint_values = req.position
	#newJointConfig = [0,-90,0,-90,0,0]


	if(synchronize_unity):
		print(f"Syncing Unity robot with {[math.degrees(value) for value in joint_values]}...")
		data = Float64MultiArray()
		data.data = [math.degrees(value) for value in joint_values]
		p.publish(data)
	else:
		print(f"Syncing Unity robot with {joint_values}...")
		print(joint_values)
		data = [math.radians(value) for value in joint_values]
		cmd_str = f"movej({data}, a=1.4, v=1.05, t=0, r=0)"
		p.publish(cmd_str)
		#rosservice call /ur_hardware_interface/dashboard/play
		#ser = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
		#ser()
	rospy.signal_shutdown("Finished sync.")


def synchronize_server():
	rospy.init_node('ur_sync',anonymous=True)
	s = rospy.Subscriber(SUB_NAME, JointState, synchronize_robots)
	
	rospy.spin()

if __name__ == "__main__":
	synchronize_server()