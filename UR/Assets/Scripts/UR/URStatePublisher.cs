using System.Collections;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.UrMoveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;


public class URStatePublisher : MonoBehaviour {
    // ROS Connector
    private ROSConnection ros;

    // Hardcoded variables 
    private int numRobotJoints = 6;
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 0.5f;
    private readonly float gripperAngle = 14f;
    // Offsets to ensure gripper is above grasp points
    private readonly Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
    private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
    // Multipliers correspond to the URDF mimic tag for each joint
    private float[] multipliers = new float[] { 1f, 1f, -1f, -1f, 1f, -1f };
    // Orientation is hardcoded for this example so the gripper is always directly above the placement object
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f,-0.5f,0.5f,-0.5f);

    // Variables required for ROS communication
	public string rosTopicName = "UR_Q";
	public float publishMessageFrequency = 0.5f;
	private float timeElapsed;

    public GameObject robot;
    public GameObject target;
    public Transform goal;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    private List<ArticulationBody> gripperJoints;



	void Start() {
		ros = ROSConnection.instance; // start the ROS connection
	}

	/// <summary>
	///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
	///     Find all gripper joints and assign them to their respective articulation body objects.
	/// </summary>
	void Awake() {
		jointArticulationBodies = new ArticulationBody[numRobotJoints];
		string shoulder_link = "world/base_link/shoulder_link";
		jointArticulationBodies[0] = robot.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

		string arm_link = shoulder_link + "/upper_arm_link";
		jointArticulationBodies[1] = robot.transform.Find(arm_link).GetComponent<ArticulationBody>();

		string elbow_link = arm_link + "/forearm_link";
		jointArticulationBodies[2] = robot.transform.Find(elbow_link).GetComponent<ArticulationBody>();

		string forearm_link = elbow_link + "/wrist_1_link";
		jointArticulationBodies[3] = robot.transform.Find(forearm_link).GetComponent<ArticulationBody>();

		string wrist_link = forearm_link + "/wrist_2_link";
		jointArticulationBodies[4] = robot.transform.Find(wrist_link).GetComponent<ArticulationBody>();

		string hand_link = wrist_link + "/wrist_3_link";
		jointArticulationBodies[5] = robot.transform.Find(hand_link).GetComponent<ArticulationBody>();

		articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

		var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
		gripperJoints = new List<ArticulationBody>();

		foreach (ArticulationBody articulationBody in robot.GetComponentsInChildren<ArticulationBody>()) {
			if (gripperJointNames.Contains(articulationBody.name)) {
				gripperJoints.Add(articulationBody);
			}
		}
	}
    
	/// <summary>
	///     Get the current values of the robot's joint angles.
	/// </summary>
	/// <returns>URMoveitJoints</returns>
	MURMoveitJoints CurrentJointConfig() {
		MURMoveitJoints joints = new MURMoveitJoints();

		joints.joint_00 = jointArticulationBodies[0].xDrive.target;
		joints.joint_01 = jointArticulationBodies[1].xDrive.target;
		joints.joint_02 = jointArticulationBodies[2].xDrive.target;
		joints.joint_03 = jointArticulationBodies[3].xDrive.target;
		joints.joint_04 = jointArticulationBodies[4].xDrive.target;
		joints.joint_05 = jointArticulationBodies[5].xDrive.target;

		return joints;
	}
	private void Update() {
		timeElapsed += Time.deltaTime;
		if (timeElapsed > publishMessageFrequency) {
            MURMoveitJoints sourceDestinationMessage = CurrentJointConfig();
            
			// Finally send the message to server_endpoint.py running in ROS
			ros.Send(rosTopicName, sourceDestinationMessage);

			timeElapsed = 0;
		}
	}
}
