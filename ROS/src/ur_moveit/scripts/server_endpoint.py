#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from ur_moveit.msg import *
from ur_moveit.srv import *
from control_msgs.msg import FollowJointTrajectoryActionGoal
from sensor_msgs.msg import Image, JointState, CompressedImage
from std_msgs.msg import Float64MultiArray
#from geometry_msgs.msg import Pose

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        #'ur_publisher': RosPublisher('ur_publisher', URMoveitJoints, queue_size=10),
        #'pose_estimation_srv': RosService('pose_estimation_service', PoseEstimationService),

        'ur_unity_joints_pub': RosPublisher('ur_unity_joints_pub', JointState, queue_size=10),
        'ur_unity_joints_sub': RosSubscriber('ur_unity_joints_sub', Float64MultiArray, tcp_server),
        'ur_moveit': RosService('ur_moveit', MoverService),
        '/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal': RosSubscriber('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, tcp_server),
        'ur_action_client': RosPublisher('ur_action_client', MoverServiceRequest, queue_size=10),
        'ur_visual_obs': RosPublisher('ur_visual_obs', CompressedImage, queue_size=1),
        'ur_vector_obs': RosPublisher('ur_vector_obs', AgentState, queue_size=10)
    })

    rospy.spin()


if __name__ == "__main__":
    main()
