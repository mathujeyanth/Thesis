using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEditor;

//using Unity.Robotics.ROSTCPConnector;
//using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.UrMoveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;



public class URControl : MonoBehaviour
{
	[RuntimeInitializeOnLoadMethod]
	private static void TurnOffLoggerInBuild()
	{
		Unity.Simulation.TimeLogger.logWallTime = false;
		Unity.Simulation.TimeLogger.logFrameTiming = false;
		Unity.Simulation.TimeLogger.logSimulationTime = false;
		Unity.Simulation.TimeLogger.logUnscaledSimulationTime = false;
		Unity.Simulation.TimeLogger.logWallTime = false;
	}
	// Hardcoded variables 
	private int NUM_ROBOT_JOINTS = 6;
	private float jointMaxSpeed = 180;
	private float JOINT_ASSIGNMENT_WAIT = 0.06f;
	private readonly float poseAssignmentWait = 0.5f;
	private readonly float gripperAngle = 14f;
	// Offsets to ensure gripper is above grasp points
	private readonly Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
	private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);

	GameObject robot;

	// Articulation Bodies
	private ArticulationBody[] jointArticulationBodies;
	ArticulationBody[] articulationChain;
	private List<ArticulationBody> gripperJoints;

	//// Utility
	//public bool executeJoints = false;
	//public bool closeGripper = false;
	//public bool resetPosition = false;
	//public double[] GetJoints = new double[6];
	//public double[] GetJointsV = new double[6];
	//public bool setPosition = false;
	//public double[] SetJoints = new double[6] { 0, 0, 0, 0, 0, 0 };

	// Joint Configs
	public double[] upConfig {get {return new double[]{0f,-90f,0f,-90f,0f,0f};}}
	public double[] zeroConfig {get {return new double[]{0f, 0f,0f,0f,0f,0f};}}

	/*Unity*/
	/// <summary>
	///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
	///     Find all gripper joints and assign them to their respective articulation body objects.
	/// </summary>
	void Awake()
	{
		robot = gameObject;
		jointArticulationBodies = new ArticulationBody[NUM_ROBOT_JOINTS];
		string shoulder_link = "world/base_link/base_link_inertia/shoulder_link";
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



	}
	[ReadOnly] public float stiffness=100000;
	[ReadOnly] public float damping=1000;
	[ReadOnly] public float forceLimit=1000;
	[ReadOnly] public float speed = 5f; // Units: degree/s
	[ReadOnly] public float torque = 100f; // Units: Nm or N
	[ReadOnly] public float acceleration = 5f;// Units: m/s^2 / degree/s^2
	void Start()
	{
		//articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();
		
		robot.gameObject.AddComponent<RosSharp.Control.FKRobot>();
		articulationChain = robot.GetComponentsInChildren<ArticulationBody>();
		float defDyanmicVal = 0.05f;
		foreach (ArticulationBody joint in articulationChain)
		{
			joint.gameObject.AddComponent<JointControl>();
			joint.jointFriction = defDyanmicVal;
			joint.angularDamping = defDyanmicVal;
			joint.maxJointVelocity = jointMaxSpeed;
			ArticulationDrive currentDrive = joint.xDrive;
			currentDrive.stiffness = stiffness;
			currentDrive.damping = damping;
			currentDrive.forceLimit = forceLimit;
			joint.xDrive = currentDrive;
		}

	}
	//private void FixedUpdate()
	//{
	//	//GetJoints = GetRobotPosition();

	//	////StartCoroutine(SetV(SetJoints));
	//	//SetV(SetJoints);
	//	//if (executeJoints)
	//	//{
	//	//	StartCoroutine(IterateToGrip(closeGripper));
	//	//	if (resetPosition)
	//	//	{
	//	//		StartCoroutine(MoveToInitialPosition());
	//	//	}
	//	//	executeJoints = false;
	//	//}
	//}

	/*UR methods*/
	/// <summary>
	///     Get the current values of the robot's joint angles.
	/// </summary>
	/// <returns>double[]</returns>
	public double[] GetJointConfig()
	{
		double[] joints = new double[NUM_ROBOT_JOINTS];
		for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		{
			joints[i] = (double)jointArticulationBodies[i].jointPosition[0]*Mathf.Rad2Deg;
			
		}
		//URMoveitJoints joints = new URMoveitJoints();
		//joints.joint_00 = jointArticulationBodies[0].xDrive.target;
		//joints.joint_01 = jointArticulationBodies[1].xDrive.target;
		//joints.joint_02 = jointArticulationBodies[2].xDrive.target;
		//joints.joint_03 = jointArticulationBodies[3].xDrive.target;
		//joints.joint_04 = jointArticulationBodies[4].xDrive.target;
		//joints.joint_05 = jointArticulationBodies[5].xDrive.target;

		return joints;
	}

	public IEnumerator MoveToInitialPosition()
	{
		bool isRotationFinished = false;
		while (!isRotationFinished)
		{
			isRotationFinished = ResetRobotToDefaultPosition();
			yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
		}
	}

	public bool ResetRobotToDefaultPosition()
	{
		float[] desiredConfiguration=new float[]{0f,-90f,0f,-90f,0f,0f};
		bool isRotationFinished = true;
		var rotationSpeed = jointMaxSpeed;

		for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		{
			var tempXDrive = jointArticulationBodies[i].xDrive;
			float currentRotation = tempXDrive.target;

			float rotationChange = rotationSpeed * Time.fixedDeltaTime;

			if (currentRotation > desiredConfiguration[i]) rotationChange *= -1;

			if (Mathf.Abs(currentRotation-desiredConfiguration[i])<rotationChange)//Mathf.Abs(currentRotation) < rotationChange)
				rotationChange = 0;
			else
				isRotationFinished = false;

			// the new xDrive target is the currentRotation summed with the desired change
			float rotationGoal = currentRotation + rotationChange;
			tempXDrive.target = rotationGoal;
			jointArticulationBodies[i].xDrive = tempXDrive;
		}

		return isRotationFinished;
		//jointArticulationBodies[0].jointPosition[0]
	}
	public IEnumerator SetJointPosition(double[] jointPosition)
	{
		//float[] desiredConfiguration=new float[]{0f,-90f,0f,-90f,0f,0f};
		for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		{
			var tmpDrive = jointArticulationBodies[i].xDrive;
			tmpDrive.target = (float)jointPosition[i];
			var tmpJointPosition = jointArticulationBodies[i].jointPosition;
			tmpJointPosition[0]=(float)jointPosition[i]*Mathf.Deg2Rad;

			jointArticulationBodies[i].xDrive=tmpDrive;
			jointArticulationBodies[i].jointPosition=tmpJointPosition;
		}
		yield return true;
	}
	public IEnumerator MoveJointPosition(double[] desiredJointConfigs)
	{
		//float[] desiredJointConfigs=new float[]{0f,-90f,0f,-90f,0f,0f};
		for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		{
			var drive = jointArticulationBodies[i].xDrive;
			drive.target = (float)desiredJointConfigs[i];
			jointArticulationBodies[i].xDrive=drive;
		}
		//bool finished=false;
		//while (!finished)
		//{
		//	finished=true;
		//	for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		//	{
		//		float jointConfig = jointArticulationBodies[i].jointPosition[0]*Mathf.Rad2Deg;
		//		if(Mathf.Abs(jointConfig-desiredJointConfigs[i])>0.001f) finished=false;
		//	}
		//	yield return new WaitForSeconds(Time.fixedDeltaTime);
		//}
		yield return true;
	}
	public IEnumerator MoveJointVelocity(double[] desiredJointConfigs)
	{
		//float[] desiredJointConfigs=new float[]{0f,-90f,0f,-90f,0f,0f};
		for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
		{
			var drive = jointArticulationBodies[i].xDrive;
			drive.target+= (float)desiredJointConfigs[i];
			jointArticulationBodies[i].xDrive=drive;
		}
		yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);	
	}



	///// <summary>
	/////
	/////     Modified from ExecuteTrajectories
	/////
	/////     Executing a single position from a possible trajectory.
	///// 
	///// </summary>
	///// <param name="jointPositions"> jointPositions recieved from a RobotTrajectory</param>
	///// <returns></returns>
	//private IEnumerator SetRobotPosition(double[] jointPositions)
	//{
	//	float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

	//	// Set the joint values for every joint
	//	for (int joint = 0; joint < NUM_ROBOT_JOINTS; joint++)
	//	{
	//		var joint1XDrive = jointArticulationBodies[joint].xDrive;
	//		joint1XDrive.target = result[joint];
	//		jointArticulationBodies[joint].xDrive = joint1XDrive;
	//	}
	//	// Wait for robot to achieve pose for all joint assignments
	//	yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
	//}

	///// <summary>
	/////
	/////     Modified from ExecuteTrajectories
	/////
	/////     Executing a single position from a possible trajectory.
	///// 
	///// </summary>
	///// <param name="jointPositions"> jointPositions recieved from a RobotTrajectory</param>
	///// <returns></returns>
	////private IEnumerator SetV(double[] jointVelocities) {
	//private bool SetV(double[] jointVelocities)
	//{

	//	for (int i = 0; i < NUM_ROBOT_JOINTS; i++)
	//	{
	//		var tempXDrive = jointArticulationBodies[i].xDrive;
	//		float currentRotation = tempXDrive.target;

	//		float rotationChange = Mathf.Clamp((float)jointVelocities[i] * Mathf.Rad2Deg, -jointMaxSpeed, jointMaxSpeed) * Time.fixedDeltaTime;

	//		// the new xDrive target is the currentRotation summed with the desired change
	//		float rotationGoal = Mathf.Clamp(currentRotation + rotationChange, tempXDrive.lowerLimit, tempXDrive.upperLimit);
	//		GetJointsV[i] = (rotationGoal - currentRotation) * Mathf.Deg2Rad / Time.fixedDeltaTime;
	//		tempXDrive.target = rotationGoal;
	//		jointArticulationBodies[i].xDrive = tempXDrive;
	//	}
	//	//yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
	//	return true;
	//}


}