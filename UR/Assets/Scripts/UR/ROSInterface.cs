using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.IO;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
//using RosMessageTypes.Moveit;
using RosMessageTypes.UrMoveit;
using RosMessageTypes.Control;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
//using RosMessageTypes.Actionlib;
//using Pose = RosMessageTypes.Geometry.Pose;
//using RosQuaternion = RosMessageTypes.Geometry.Quaternion;

public class ROSInterface : MonoBehaviour
{
	public UnityEngine.UI.Text text;
	public bool planMotion = false;
	public bool resetRobot = false;
	public bool publishJoints = false;
	public bool setGripValue = false;
	public float gripValue = 0f;

	// ROS
	private ROSConnection ros;
	[SerializeField, ReadOnly] string SERVICE_MOVEIT = "ur_moveit";
	[SerializeField, ReadOnly] string PUB_JOINTS = "ur_unity_joints_pub";
	[SerializeField, ReadOnly] string SUB_JOINTS = "ur_unity_joints_sub";
	[SerializeField, ReadOnly] string PUB_ACTIONCLIENT = "ur_action_client";
	[SerializeField, ReadOnly] string PUB_VISUAL_OBS = "ur_visual_obs";
	[SerializeField, ReadOnly] string PUB_VECTOR_OBS = "ur_vector_obs";
	[SerializeField, ReadOnly] string SUB_FOLLOWJOINT = "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal";
	bool isActionCompleted = false;

	// Hardcoded Robot variables
	private float JOINT_ASSIGNMENT_WAIT = 0.06f;
	private float POSE_ASSIGNMENT_WAIT = 0.5f;
	private float gripperAngle = 14f;
	// Offsets to ensure gripper is above grasp points
	private Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
	private Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
	// Orientation is hardcoded for this example so the gripper is always directly above the placement object
	private readonly Quaternion pickOrientation = new Quaternion(-0.5f, -0.5f, 0.5f, -0.5f);

	public GameObject robot;
	public Transform pick;
	public Transform middle;
	public Transform place;
	
	//public Transform goal;

	// Articulation Bodies
	private ArticulationBody[] jointArticulationBodies;
	ArticulationBody[] articulationChain;
	private List<ArticulationBody> gripperJoints;

	/*Robot*/
	URControl urControl;

	/*Gripper*/
	GripperControl gripControl;
	bool movingLock = false;

	/*Camera*/
	CameraUtility camUtility;

	/*Agent*/
	public LEGO brick;
	public Transform GripCenter;
	public Transform GripStart;
	public GameObject placeArea;
	bool executedPath = false;


	//private RenderTexture renderTexture;

	//public PoseEstimationScenario scenario;

	private enum Poses
	{
		PreGrasp,
		Grasp,
		PickUp,
		Place,
		PostPlace
	};

	/// <summary>
	///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
	///     Find all gripper joints and assign them to their respective articulation body objects.
	/// </summary>
	void Awake()
	{		
		// Get ROS connection static instance
		ros = ROSConnection.instance;
		var sr = new StreamReader(Application.dataPath + "/" + "config.txt");
		var fileContents = sr.ReadToEnd();
		sr.Close();
		var lines = fileContents.Split('\n');
		if(lines[0].Trim() == "1"){
			ros.RosIPAddress =  lines[1].Trim();
			//ip = lines[1];
			//ros.overrideUnityIP = lines[2];
		}
	}

	void Start()
	{
		urControl = robot.GetComponent<URControl>();
		gripControl = robot.GetComponent<GripperControl>();
		camUtility = gameObject.GetComponent<CameraUtility>();

		//Debug.Log($"{ros.overrideUnityIP}:{ros.unityPort}");

		//// Render texture
		//renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
		//renderTexture.Create();

		StartCoroutine(PublishRequest(0.008f));//125Hz stated in doc, should fit with Time.fixedDelta

		ros.Subscribe<RosMessageTypes.Std.MFloat64MultiArray>(SUB_JOINTS, MoveJointsTopic);
		ros.Subscribe<MFollowJointTrajectoryActionGoal>(SUB_FOLLOWJOINT, ExecuteRobotTrajectory);

		ros.Connect(ros.RosIPAddress,ros.RosPort);
		Debug.Log($"{ros.RosIPAddress}:{ros.RosPort}");

		brick.TargetPosition = placeArea.transform.position;
		brick.TargetPosition.y += brick.height;
		brick.TargetRotation = Quaternion.Euler(0f, 0f, 0f);
	}

	void FixedUpdate() {

		if (planMotion)
		{
			planMotion = false;
			//URMoveitJoints Q = CurrentJointConfig();
			//Debug.Log(Q.ToString());
			TrajectoryService();
			//Initialize();
		}
		if(publishJoints)
		{
			publishJoints=false;
			PublishJoints(middle.position,middle.rotation);
		}
		if(resetRobot)
		{
			resetRobot=false;
			//StartCoroutine(MoveToInitialPosition());
			StartCoroutine(urControl.SetJointPosition(urControl.upConfig));
			//urControl.SetJointPosition(urControl.upConfig);
			//StartCoroutine(MoveJointPosition(new double[]{0f,-90f,0f,-90f,0f,0f}));
		}
		if(setGripValue)
		{
			setGripValue=false;
			gripControl.setGrip(gripValue);
		}
	}
	/*GUI*/

	/// <summary>
	///     Button callback for setting the robot to default position
	/// </summary>
	public void Reset()
	{
		gripControl.setGrip(0f);
		StartCoroutine(urControl.MoveToInitialPosition());
	}
	public void ForceReset()
	{
		gripControl.setGrip(0f);
		StartCoroutine(urControl.SetJointPosition(urControl.upConfig));
		//urControl.SetJointPosition(urControl.upConfig);
	}
	public void GoToPick()
	{
		PublishJoints(pick.position-Vector3.up*0.055f,pick.rotation);
	}
	
	public void PickUpObject()
	{
		//isActionCompleted = false;
		//PublishJoints(pick.position-Vector3.up*0.055f,pick.rotation);
		executedPath=false;
		PublishJoints(brick.transform.position,pick.rotation);
		StartCoroutine(GraspObject());//executedPath
		//gripControl.setGrip(1f);
	}
	
	public void PlaceObject()
	{
		gripControl.setGrip(0f);
	}
	
	public void GoToMiddle()
	{
		PublishJoints(middle.position-Vector3.up*0.055f,middle.rotation);
	}
	public void GoToPlace()
	{
		PublishJoints(place.position-Vector3.up*0.055f,place.rotation);
	}
	public void ResetBrick()
	{
		//StartCoroutine(OpenGrip());
		/*Grasp brick coroutine*/
		StartCoroutine(CloseGrip());

	}

	/// <summary>
    ///     Execute trajectories from RobotMoveActionGoal topic.
    ///
    ///     Execution will iterate through every robot pose in the trajectory pose
    ///     array while updating the joint values on the simulated robot.
    ///
    /// </summary>
    /// <param name="trajectory"> The array of poses for the robot to execute</param>
    private IEnumerator ExecuteTrajectory(RosMessageTypes.Trajectory.MJointTrajectory joint_trajectory)
    {
        // For every robot pose in trajectory plan
        foreach (var point in joint_trajectory.points)
        {
            var jointPositions = point.positions;
            double[] result = jointPositions.Select(r=> (double)r * Mathf.Rad2Deg).ToArray();

            //// Set the joint values for every joint
            //for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            //{
            //    var joint1XDrive  = jointArticulationBodies[joint].xDrive;
            //    joint1XDrive.target = result[joint];
            //    jointArticulationBodies[joint].xDrive = joint1XDrive;
            //}
			StartCoroutine(urControl.MoveJointPosition(result));
			//urControl.MoveJointPosition(result);
            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
        }
		executedPath = true;
    }

	//void ExecuteRobotCommands(RobotMoveActionGoal robotAction)
    //{

    //}

	/*ROS Publishers*/
	public void PublishJoints(Vector3 targetPosition, Quaternion targetOrientation)// Transform transform)
	{
		MMoverServiceRequest request = new MMoverServiceRequest();
		double[] currentJoints = urControl.GetJointConfig();
		request.joints_input = new MURMoveitJoints{
			joint_00=currentJoints[0],
			joint_01=currentJoints[1],
			joint_02=currentJoints[2],
			joint_03=currentJoints[3],
			joint_04=currentJoints[4],
			joint_05=currentJoints[5]
		};

		// Pick Pose
		request.pick_pose = new RosMessageTypes.Geometry.MPose
		{
			position = (targetPosition-Vector3.up*0.77f).To<FLU>(),
			
			orientation = targetOrientation.To<FLU>()
		};

		//// Place Pose
		//request.place_pose = new RosMessageTypes.Geometry.MPose
		//{
		//	position = (goal.position+placePoseOffset).To<FLU>(),
		//	orientation = pickOrientation.To<FLU>()//goal.rotation.To<FLU>()
		//};

		//ros.SendServiceMessage<MoverServiceResponse>(ROS_SERVICE_NAME, request, TrajectoryResponse);
		ros.Send(PUB_ACTIONCLIENT, request);
	}

	/*ROS Subscribers*/

	/*ROS Services*/

	//public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
	//{
	//	MoverServiceRequest request = new MoverServiceRequest();
	//	request.joints_input = CurrentJointConfig();

	//	// Pick Pose
	//	request.pick_pose = new RosMessageTypes.Geometry.Pose
	//	{
	//		position = (targetPos + pickPoseOffset).To<FLU>(),
	//		orientation = Quaternion.Euler(90, targetRot.eulerAngles.y, 0).To<FLU>()
	//	};

	//	// Place Pose
	//	request.place_pose = new RosMessageTypes.Geometry.Pose
	//	{
	//		position = (goal.position + placePoseOffset).To<FLU>(),
	//		orientation = pickOrientation.To<FLU>()
	//	};

	//	ros.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
	//}
	public void TrajectoryService()
	{
		MMoverServiceRequest request = new MMoverServiceRequest();
		double[] currentJoints = urControl.GetJointConfig();
		request.joints_input = new MURMoveitJoints{
			joint_00=currentJoints[0],
			joint_01=currentJoints[1],
			joint_02=currentJoints[2],
			joint_03=currentJoints[3],
			joint_04=currentJoints[4],
			joint_05=currentJoints[5]
		};

		// Pick Pose
		request.pick_pose = new RosMessageTypes.Geometry.MPose
		{
			position = (middle.position-Vector3.up*0.77f).To<FLU>(),
			orientation = middle.rotation.To<FLU>()
		};

		//// Place Pose
		//request.place_pose = new RosMessageTypes.Geometry.MPose
		//{
		//	position = (goal.position+placePoseOffset).To<FLU>(),
		//	orientation = pickOrientation.To<FLU>()//goal.rotation.To<FLU>()
		//};

		ros.SendServiceMessage<MMoverServiceResponse>(SERVICE_MOVEIT, request, TrajectoryResponse);
	}

	void TrajectoryResponse(MMoverServiceResponse response)
	{
		if (response.trajectories != null && response.trajectories.Length > 0)
		{
			StartCoroutine(ExecuteTrajectories(response));
		}
		else
		{
			Debug.Log("Failed");
		}
	}

	/// <summary>
	///     Execute the returned trajectories from the MoverService.
	///
	///     The expectation is that the MoverService will return four trajectory plans,
	///         PreGrasp, Grasp, PickUp, and Place,
	///     where each plan is an array of robot poses. A robot pose is the joint angle values
	///     of the six robot joints.
	///
	///     Executing a single trajectory will iterate through every robot pose in the array while updating the
	///     joint values on the robot.
	///
	/// </summary>
	/// <param name="response"> MoverServiceResponse received from ur_moveit mover service running in ROS</param>
	/// <returns></returns>
	private IEnumerator ExecuteTrajectories(MMoverServiceResponse response)
	{
		if (response.trajectories != null)
		{
			// For every trajectory plan returned
			for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
			{
				// For every robot pose in trajectory plan
				yield return ExecuteTrajectory(response.trajectories[poseIndex].joint_trajectory);

				// Close the gripper if completed executing the trajectory for the Grasp pose
				if (poseIndex == (int)Poses.Grasp)
				{
					//StartCoroutine(IterateToGrip(true));
					//yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
					gripControl.setGrip(1);
					yield return true; //IterateToGrip(true);
				}
				else if (poseIndex == (int)Poses.Place)
				{
					//yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
					//// Open the gripper to place the target cube
					//StartCoroutine(IterateToGrip(false));
					gripControl.setGrip(0);
					yield return true;//IterateToGrip(false);
				}
				// Wait for the robot to achieve the final pose from joint assignment
				yield return new WaitForSeconds(POSE_ASSIGNMENT_WAIT);
			}

			yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
		}
		else
		{
			Debug.Log("failed");
		}
	}

    /// <summary>
    ///   Execute robot commands receved from ROS Subscriber.
    ///   Gripper commands will be executed immeditately wihle trajectories will be
    ///   executed in a coroutine.
    /// </summary>
    /// <param name="robotAction"> RobotMoveActionGoal of trajectory or gripper commands</param>
    void ExecuteRobotTrajectory(MFollowJointTrajectoryActionGoal robotAction)
    {
		//if (response.trajectories != null && response.trajectories.Length > 0)
		//{
		//	response.trajectories[0].joint_trajectory.points
		//	StartCoroutine(ExecuteTrajectories(response));
		//}
		//else
		//{
		//	Debug.Log("Failed");
		//}

		//Debug.Log("Received");
		if (robotAction.goal != null)
		{
			StartCoroutine(ExecuteTrajectory(robotAction.goal.trajectory));
		}
		else
		{
			Debug.Log("Failed");
		}
    }
	void MoveJointsTopic(RosMessageTypes.Std.MFloat64MultiArray joints){
		//Debug.Log(joints.data.Length);
		//StartCoroutine(urControl.SetJointPosition(joints.data));
		//double[] ur_joints = new double[]{
		//	joints.data[0],
		//	joints.data[1],
		//	joints.data[2],
		//	joints.data[3],
		//	joints.data[4],
		//	joints.data[5]
		//};
		//Debug.Log(ur_joints);
		//StartCoroutine(urControl.MoveJointVelocity(joints.data));
		StartCoroutine(urControl.MoveJointPosition(joints.data));
	}
	IEnumerator OpenGrip(){
		//while (movingLock)
		//{
		//	yield return new WaitForFixedUpdate();
		//}
		movingLock = true;
		gripControl.setGrip(0);
		while (gripControl.getGrip()==0f)
		{
			yield return new WaitForFixedUpdate();
		}
		movingLock = false;
		Debug.Log("opened");
		yield return new WaitForFixedUpdate();
	}
	IEnumerator CloseGrip(){
		//while (movingLock)
		//{
		//	yield return new WaitForFixedUpdate();
		//}
		//movingLock = true;
		yield return OpenGrip();

		brick.rb.useGravity=false;
		brick.rb.isKinematic = true;
		brick.transform.position= GripCenter.position;
		brick.transform.rotation = Quaternion.identity;//GripCenter.transform.rotation* Quaternion.Euler(0,0,180f);
		/*Check if cube is grasped*/
		brick.rb.isKinematic=false;
		while (!gripControl.isGrasping())
		{
			gripControl.moveGrip(-1f);
			//Debug.Log("Closing");
			yield return new WaitForFixedUpdate();
		}
		brick.rb.useGravity = true;
		//brick.rb.position = GripCenter.position; //startPosition;
		//brick.rb.rotation = GripCenter.rotation; //startRotation;
		//Debug.Log("finished");
		movingLock = false;
		Debug.Log("closed");
		yield return new WaitForFixedUpdate();
		//resetState = true;
	}
	IEnumerator GraspObject(){
		while (!executedPath)
		{
			yield return new WaitForFixedUpdate();
		}
		gripControl.setGrip(1);
	}
	MAgentState GetObservations()
	{
		//rosImage.header.stamp.secs = (uint)Time.realtimeSinceStartup;
		//state.header.stamp.secs = (uint)Time.realtimeSinceStartup;
		MAgentState state = new MAgentState();
		Vector3 gripperPosition = GripStart.InverseTransformPoint(GripCenter.position);
		Quaternion gripperRotation = GripCenter.rotation;

		Vector3 targetPosition = GripStart.InverseTransformPoint(brick.TargetPosition);
		Quaternion targetRotation = brick.TargetRotation;

		Vector3 gripper2target_pos = (targetPosition - gripperPosition)*100f;
		Quaternion gripper2target_rot = targetRotation.ShortestRotation(gripperRotation);

		Vector3 velocity = gripControl.velocityNormalized;
		Quaternion angularVelocity = Quaternion.Euler(gripControl.angularVelocityNormalized);
		state.grip = gripControl.getGrip();

		state.gripper2target_position = new MVector3{
			x=gripper2target_pos.x,
			y=gripper2target_pos.y,
			z=gripper2target_pos.z
		};
		state.gripper2target_rotation = new MQuaternion{
			x=gripper2target_rot.x,
			y=gripper2target_rot.y,
			z=gripper2target_rot.z,
			w=gripper2target_rot.w
		};

		state.gripper_position = new MVector3{
			x=gripperPosition.x,
			y=gripperPosition.y,
			z=gripperPosition.z
		};
		state.gripper_rotation = new MQuaternion{
			x=gripperRotation.x,
			y=gripperRotation.y,
			z=gripperRotation.z,
			w=gripperRotation.w
		};
		state.linear_velocity = new MVector3{
			x=velocity.x,
			y=velocity.y,
			z=velocity.z
		};
		state.angular_velocity = new MQuaternion{
			x=angularVelocity.x,
			y=angularVelocity.y,
			z=angularVelocity.z,
			w=angularVelocity.w
		};
		return state;
	}
	IEnumerator PublishRequest(float secondsBetweenSpawns)
	{
		while(true)
		{
			/*Joints*/
			//PublishJoints();
			MJointState jointConfig = new MJointState();
			jointConfig.position = urControl.GetJointConfig();

			/*Camera*/
			MCompressedImage rosImage = new MCompressedImage(new RosMessageTypes.Std.MHeader(), "png",camUtility.PoseEstimation());
			
			/*TCP pose and twist+grip*/
			MAgentState state = GetObservations();

			/*Send*/
			ros.Send(PUB_JOINTS,jointConfig);
			ros.Send(PUB_VISUAL_OBS, rosImage);
			ros.Send(PUB_VECTOR_OBS, state);
			yield return new WaitForSecondsRealtime(secondsBetweenSpawns);
		}
	}

}