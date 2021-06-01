using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

using System;
using System.IO;

public class RobotiqAgent : Agent {
	/*Objects*/
	public GameObject BrickContainer;
	BrickManager brickManager;
	ObjectBasic currentBrick;

	public GameObject placeArea;
	Vector3 placeAreaBounds;
	float[] BRICK_ROTATIONS = { 0f, 90f, 180f, 270f }; //TODO: as enum?

	/*Gripper*/

    public Transform GripCenter;
	public Transform GripStart;
	//Vector3 startPosition;
	//Quaternion startRotation;
	TCPControl gripper;
	//float minPrecisionValue = 0.00001f;

	/*Agent*/
	EnvironmentParameters m_ResetParams;
	int STACK_STATE;
	[ReadOnly,SerializeField] bool resetState = false;
	GripperDecisionRequester decisionRequester;

	/*Debug*/
	public bool showDebugText = false;
	public UnityEngine.UI.Text text;
	string observationText = "";
	string actionText = "";
	public UnityEngine.UI.RawImage camera;
	Texture2D texture;
	//Unity.MLAgents.Sensors.CameraSensorComponent WristCamera;
	public Camera WristCamera;
	public int DefaultNumOfBricks = 1;
	public bool DisableTargetRotation = false;
	public bool DisableGripperRotation = false;
	[ReadOnly] public int agentLayer = 6;//LayerMask.NameToLayer("Agent0"); 
	StreamWriter sr;
	int counter = 0;
	public override void Initialize() {
		WristCamera.cullingMask = 1 << agentLayer;
		//startPosition = GripCenter.position;
		//startRotation = Quaternion.identity;

		gripper = GetComponent<TCPControl>();

		brickManager = BrickContainer.GetComponent<BrickManager>();

		foreach (Transform trans in gameObject.GetComponentsInChildren<Transform>())
        {
            trans.gameObject.layer = agentLayer;
        }
		foreach (Transform trans in placeArea.GetComponentsInChildren<Transform>())
        {
            trans.gameObject.layer = agentLayer;
        }


		placeAreaBounds = placeArea.GetComponent<Collider>().bounds.extents;
		decisionRequester = GetComponent<GripperDecisionRequester>();
        //Academy.Instance.AutomaticSteppingEnabled = true;
		m_ResetParams = Academy.Instance.EnvironmentParameters;

		//WristCamera = gameObject.GetComponent<Unity.MLAgents.Sensors.CameraSensorComponent>();
		//texture = new Texture2D(WristCamera.Width, WristCamera.Height);
		//var image = WristCamera.CreateSensors()[0].GetCompressedObservation();
		//System.IO.File.WriteAllBytes($"IMG_{WristCamera.Width}x{WristCamera.Height}.png", image);
		//camera.color = new UnityEngine.Color(1, 1, 1, 1);
		sr = new StreamWriter("articulationIssue.csv");
		sr.WriteLine("step,pos,vel,acc");
	}


	//public override void OnEpisodeBegin() {
	//	//Resources.UnloadUnusedAssets();
	//	decisionRequester.grasped = false;
	//	resetState = false;
	//	STACK_STATE = (int)m_ResetParams.GetWithDefault("N_BRICKS", DefaultNumOfBricks);
	//	int N_BRICKS = STACK_STATE;
	//	if (currentBrick == null || (currentBrick != null && !currentBrick.IsCompleted) || brickManager.placedAllBricks() || brickManager.placedAndRemoved()) { // 
	//		brickManager.DestroyAllBricks(); // Destroy old

	//		Vector3 position = brickManager.pickArea.transform.position;// + Vector3.up * 0.05f;
	//		for (int i = 0; i < N_BRICKS; i++) {
	//			brickManager.AddBrick((int)BrickType.B222, (int)BrickColor.random, false);
	//			//brickManager.AddBrick(Random.Range(0, 1+1), (int)BrickColor.random, false);
	//			ObjectBasic tmpBrick = brickManager.bricksInstantiated.Last();
	//			tmpBrick.transform.position = position + Vector3.up * 0.2f; //not using rb.position, since it requires physics
	//			/*Random position on placearea for first brick*/
	//			if (i == 0) {
	//				tmpBrick.TargetPosition = placeArea.transform.position;

	//				tmpBrick.TargetPosition.y += tmpBrick.height;//0.045f;//TODO: find floor 2 brick height. Currently a static value 
	//				tmpBrick.TargetPosition.x += 0.08f * Random.Range(-4, 4); //[-0.8f,0.8f]
	//				tmpBrick.TargetPosition.z += 0.08f * Random.Range(-4, 4);
	//			}
	//			/*Place other bricks on top of the previous*/
	//			else {
	//				ObjectBasic previousBrick = brickManager.bricksInstantiated[brickManager.bricksInstantiated.Count - 2];
	//				tmpBrick.TargetPosition = previousBrick.TargetPosition;
	//				tmpBrick.TargetPosition.y += 0.1f;//TODO: find brick 2 brick height. Currently a static value
	//			}
	//			float brick_rotation = DisableTargetRotation ? 0 : BRICK_ROTATIONS[Random.Range(0, 3)];
	//			tmpBrick.TargetRotation = Quaternion.Euler(0f, brick_rotation, 0f);
	//			//tmpBrick.TargetRotation = Quaternion.identity;
	//			position = tmpBrick.transform.position;
	//		}
	//		/*TODO: Add bricks based on a list and set their targetposition*/
	//		currentBrick = null; //No previous brick
	//	}
	//	/*Initialize brick to be placed*/
	//	currentBrick = brickManager.GetCurrentBrick();
	//	foreach (Transform trans in currentBrick.GetComponentsInChildren<Transform>())
    //    {
    //        trans.gameObject.layer = agentLayer;
    //    }
	//	currentBrick.resetTag();
	//	//ScreenCapture.CaptureScreenshot("2.png", 1);
	//	//Debug.Break();
	//	/*Reset Gripper*/
	//	StartCoroutine(ResetGripperCheck());

	//}
	//IEnumerator ResetGripperCheck(){
	//	yield return new WaitForFixedUpdate();
	//	/*Reset Gripper*/
	//	gripper.resetGraspCollider();
	//	float rotationY = DisableGripperRotation ? 0 : new float[]{-1f,1f}[Random.Range(0, 1)]*BRICK_ROTATIONS[Random.Range(0, 3)];
	//	gripper.setTCP(Vector3.zero,Vector3.up*rotationY); //TODO: not all rotations, only one direction

	//	gripper.setGrip(0f);

	//	/*Check if gripper is reset (standing still)*/
	//	while (!gripper.isGripperSleeping())
	//	{
	//		//Debug.Log("begun");
	//		yield return new WaitForFixedUpdate();
	//	}
	//	//ScreenCapture.CaptureScreenshot("3.png", 1);
	//	//Debug.Break();
	//	//yield return new WaitForFixedUpdate();
	//	currentBrick.rb.isKinematic = true;
	//	currentBrick.rb.useGravity = false;
	//	currentBrick.rb.position = GripCenter.position;//startPosition;
	//	currentBrick.rb.rotation = GripCenter.rotation;//startRotation;
	//	//GripStart.position = GripCenter.position;
	//	//GripStart.rotation = GripCenter.rotation;
	//	//Debug.Log($"Cube set: {cube.transform.position}");
	//	yield return new WaitForFixedUpdate();

	//	/*Check if cube has been placed*/
	//	while(currentBrick.rb.position != GripCenter.position/*startPosition*/ || currentBrick.rb.rotation != GripCenter.rotation/*startRotation*/ || !currentBrick.rb.IsSleeping())
	//	{
	//		yield return new WaitForFixedUpdate();
	//	}
	//	//ScreenCapture.CaptureScreenshot("4.png", 1);
	//	//Debug.Break();
	//	//yield return new WaitForFixedUpdate();
	//	//Debug.Log($"Cube pose check: {cube.transform.position}");
	//	currentBrick.rb.isKinematic = false;
	//	yield return new WaitForFixedUpdate();

	//	/*Check if cube is grasped*/
	//	while (!gripper.isGrasping())
	//	{
	//		gripper.moveGrip(-1f);
	//		//Debug.Log("Closing");
	//		yield return new WaitForFixedUpdate();
	//	}
	//	yield return new WaitForFixedUpdate();
	//	currentBrick.rb.useGravity = true;
	//	//ScreenCapture.CaptureScreenshot("5.png", 1);
	//	//Debug.Break();
	//	//yield return new WaitForFixedUpdate();
	//	//Debug.Log($"Grasped: {cube.transform.position}");
	//	//currentBrick.rb.position = GripCenter.position; //startPosition;
	//	//currentBrick.rb.rotation = GripCenter.rotation; //startRotation;
	//	//Debug.Log("finished");
	//	yield return new WaitForFixedUpdate();
	//	resetState = true;
	//	decisionRequester.grasped = true;
	//}
	//IEnumerator ResetGripperCheckFirst(){
	//	/*Check if gripper is reset (standing still)*/
	//	while (!gripper.isGripperSleeping())
	//	{
	//		//Debug.Log("begun");
	//		yield return new WaitForFixedUpdate();
	//	}
	//	yield return new WaitForFixedUpdate();
	//}
	//public override void CollectObservations(VectorSensor sensor) {
	//	//sensor.AddObservation(0f);
	//	float grip = gripper.getGrip();
	//	//Vector3 gripperCenter = gripperController.GetGripperCenterPosition();

	//	//Vector3 TCPposition = TCP.getPosition();
	//	//Quaternion TCProtation = Quaternion.Euler(TCP.getRotation());

	//	Vector3 TCPlinearVelocity = gripper.velocityNormalized;
	//	Quaternion TCPangularVelocity = Quaternion.Euler(gripper.angularVelocityNormalized);

	//	Vector3 gripperPosition = GripStart.InverseTransformPoint(GripCenter.position);
	//	Quaternion gripperRotation = GripCenter.rotation;
	//	Vector3 targetPosition = GripStart.InverseTransformPoint(currentBrick.TargetPosition);
	//	Quaternion targetRotation = currentBrick.TargetRotation;

	//	string info = "Observations:\n";
	//	Vector3 vec3;
	//	Quaternion quat4;

	//	/*Gripper: 1*/
	//	float f1 = grip;
	//	if (showDebugText) info += "Grip: " + f1.ToString("F3") + "\n";
	//	sensor.AddObservation(f1);

	//	/*GripCenter to Brick Target distance: 7*/
	//	vec3 = (targetPosition - gripperPosition);
	//	quat4 = targetRotation.ShortestRotation(gripperRotation);
	//	//sensor.AddObservation(vec3);
	//	sensor.AddObservation(vec3*100f);
	//	sensor.AddObservation(quat4);
	//	if (showDebugText) info += "TCP to Brick: " + vec3.ToString("F3") + " | " + quat4.eulerAngles.EulerLimit().ToString("000") +"\n";

	//	/*GripCenter to GripStart distance: 7*/
	//	vec3 = gripperPosition;
	//	quat4 = gripperRotation;
	//	sensor.AddObservation(vec3);
	//	sensor.AddObservation(quat4);
	//	if (showDebugText) info += "TCP: " + vec3.ToString("F3") + " | " + quat4.eulerAngles.EulerLimit().ToString("000") +"\n";

	//	/*TCP Velocity: 7*/
	//	sensor.AddObservation(TCPlinearVelocity);
	//	sensor.AddObservation(TCPangularVelocity);

	//	/*Forces: 6*/
	//	//Add forces measured by tcp (and gripper fingers if possible)

	//	/*Brick and TCP distance: 7*/
	//	Vector3 brickPosition = GripStart.InverseTransformPoint(currentBrick.rb.position);
	//	Quaternion brickRotation = currentBrick.rb.rotation;

	//	//vec3 = gripperPosition-brickPosition;
	//	//quat4 = targetRotation.ShortestRotation(brickRotation); 
	//	//sensor.AddObservation(vec3);
	//	//sensor.AddObservation(quat4);
	//	//if (showDebugText) info+="G2B: "+vec3.ToString("F3")+" | "+quat4.eulerAngles.EulerLimit().ToString("F3")+"\n";

	//	///*Brick and target distance: 7*/
	//	//vec3 = targetPosition - brickPosition;
	//	//quat4 = brickRotation.ShortestRotation(currentBrick.TargetRotation);
	//	//sensor.AddObservation(vec3);
	//	//sensor.AddObservation(quat4);
	//	//if (showDebugText) info+="B2T: "+vec3.ToString("F3")+" | "+quat4.eulerAngles.EulerLimit().ToString("F3")+"\n";

	//	if (showDebugText) {
	//		observationText = "Lesson: " + STACK_STATE + "\n";
	//		observationText += info + "\n";
	//		observationText += "Desired: " + targetPosition.ToString("F3") + " | " + targetRotation.eulerAngles.EulerLimit().ToString("000") +"\n";
	//		observationText += "Brick: " + brickPosition.ToString("F3") + " | " + brickRotation.eulerAngles.EulerLimit().ToString("000") +"\n";
	//		observationText += "Gripper: " + gripperPosition.ToString("F3") + " | " + gripperRotation.eulerAngles.EulerLimit().ToString("000") +"\n";
	//		//text.text = "Curriculum State: " + STACK_STATE + "\n";
	//		//text.text += info + "\n";
	//		//text.text += "Desired: " + targetPosition.ToString("F3") + " | " + targetRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
	//		//text.text += "Brick: " + brickPosition.ToString("F3") + " | " + brickRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
	//		//text.text += "Gripper: " + gripperPosition.ToString("F3") + " | " + gripperRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
	//	}
	//}
	
	//public override void OnActionReceived(ActionBuffers actions) {

	//	float[] actionsUsed = new float[7];
	//	for (int i = 0; i < 7; i++) {
	//		actionsUsed[i] = DiscreteToMoveInput(actions.DiscreteActions[i]);
	//	}

	//	if (showDebugText) actionText = "\nActions: " + string.Join(",", actionsUsed) + "\n\n";

	//	float gripperState = actionsUsed[0];
	//	gripper.moveGrip(gripperState);
	//	//if (gripperState != 0) {
	//	//	gripper.moveGrip(gripperState);
	//	//}
    //    Vector3 prismaticState = Vector3.zero;
	//	prismaticState.x = actionsUsed[1];
	//	prismaticState.y = actionsUsed[2];
	//	prismaticState.z = actionsUsed[3];

    //    Vector3 revoluteState = Vector3.zero;
	//	//revoluteState.y = actionsUsed[4];
	//	revoluteState.x = actionsUsed[4];
	//	revoluteState.y = actionsUsed[5];
	//	revoluteState.z = actionsUsed[6];

	//	gripper.moveTCP(prismaticState,revoluteState);

	//	AssignRewardsBricks();
	//	if (showDebugText) text.text = observationText + actionText;
	//}

	//void AssignRewardsBricks() {
	//	Vector3 brickPosition = currentBrick.transform.position;
	//	Quaternion brickRotation = currentBrick.transform.rotation;
	//	Vector3 distanceToPlaceArea = (placeArea.transform.position - brickPosition);
	//	Vector3 gripperPosition = GripCenter.position;
	//	float brick2target_pos = Vector3.Distance(currentBrick.TargetPosition, brickPosition);
	//	float brick2target_rot = Mathf.Abs(Quaternion.Angle(currentBrick.TargetRotation, brickRotation)) / 180;

	//	/*If brick is outside XY plane: big negative + end episode*/
	//	if (Mathf.Abs(distanceToPlaceArea.x) > placeAreaBounds.x || Mathf.Abs(distanceToPlaceArea.z) > placeAreaBounds.z) {
	//		SetReward(-1f);
	//		EndEpisode();
	//	}

	//	/*If brick and grippercenter distance too big*/
	//	if (Vector3.Distance(gripperPosition, brickPosition) > 0.5f/*5cm*/) {
	//		SetReward(-1f);
	//		EndEpisode();
	//	}

	//	/*If moving other bricks: large negative + end?*/
	//	if (brickManager.placedAndRemoved()){
	//		//SetReward(-1f);
	//		EndEpisode();
	//	}

	//	/*If brick is stacked: big positive + end episode*/
	//	bool isPlaced = currentBrick.isPlaced();
	//	bool isGrasped = gripper.isGrasping();
	//	if (isPlaced && !isGrasped && currentBrick.rb.velocity.magnitude < 0.05f) {
	//		//Debug.Log(StepCount);
	//		currentBrick.rb.constraints = RigidbodyConstraints.FreezePositionX+(int)RigidbodyConstraints.FreezePositionZ;
	//		currentBrick.IsCompleted = true;
	//		SetReward(1f);
	//		currentBrick.removeTag();
	//		EndEpisode();
	//	}

	//	/*Reward Shaping*/

	//	string info = "";
	//	float reward = 0f;
	//	float episodeReward = 0f;

	//	reward = -0.001f;
	//	AddReward(reward);//Time used penalty: small negative
	//	episodeReward += reward;
	//	if (showDebugText) info += "Time penalty: " + reward.ToString("F3") + "\n";

	//	//reward = stateSum == 0 ? -0.001f : 0f;  // Penalty for standing still
	//	//AddReward(reward);
	//	//episodeReward += reward;
	//	//if (showDebugText) info += "No Action Penalty: " + reward.ToString("F3") + "\n";

	//	bool inCollison = gripper.inCollision();
	//	reward = inCollison ? -0.01f : 0f;
	//	AddReward(reward); // Penalty for collision
	//	episodeReward += reward;
	//	if (showDebugText) info += "In collision: " + reward.ToString("F3") + "\n";

	//	//reward = isGrasped ? 0.001f * (1f - brick2target_pos * 10) : 0f;
	//	//reward = 0.001f * Mathf.Clamp01(1f - brick2target_pos * 10);
	//	//reward = 0.001f * (1f - brick2target_pos * 10f);
	//	//if (showDebugText) info += "brick2target_pos: " + brick2target_pos + "\n";
	//	//if (showDebugText) info += "brick2target_rot: " + brick2target_rot + "\n";
	//	reward = brick2target_pos <= 1f/10f ? 0.001f * (1f - brick2target_pos * 10) : -0.0001f * brick2target_pos;
	//	//reward = brick2target_pos <= 1f/10f ? 0.0006f * (1f - brick2target_pos * 10) : 0.0005f * (1f - brick2target_pos);
	//	//reward = (brick2target_pos <= 1f/10f ? 0.00005f * (1f - brick2target_pos * 10) : 0) + 0.0005f*(1f - brick2target_pos );
	//	//reward = 0.00051f * (1f - brick2target_pos);
	//	//Vector3 dist = (currentBrick.TargetPosition - brickPosition);
	//	//reward = 0.001f * (Mathf.Clamp01(1f-Mathf.Abs(dist.x))/3 + Mathf.Clamp01(1f-Mathf.Abs(dist.y))/3 + Mathf.Clamp01(1f-Mathf.Abs(dist.z))/3 );
	//	AddReward(reward);
	//	episodeReward += reward;
	//	if (showDebugText) info += "Brick position distance: " + reward.ToString("F4") + "\n";

	//	//reward = isGrasped ? 0.001f * (1f - brick2target_rot * 10) : 0f;
	//	//reward = 0.001f * Mathf.Clamp01(1f - brick2target_rot * 10);
	//	reward = brick2target_rot <= 1f/10f ? 0.001f * (1f - brick2target_rot * 10) : -0.0001f * brick2target_rot;
	//	//reward = (brick2target_rot <= 1f/10f ? 0.00005f * (1f - brick2target_rot * 10) : 0) + Mathf.Clamp(0.0005f*(1f - brick2target_rot ),-0.001f,0.0005f);
	//	//reward = 0.00051f*(1f - brick2target_rot );
	//	//reward = 0.001f * (1f - brick2target_rot * 10f);
	//	AddReward(reward);
	//	episodeReward += reward;
	//	if (showDebugText) info += "Brick rotation distance: " + reward.ToString("F4") + "\n";

	//	//reward = isPlaced ? 0.1f : 0f;  //If placed, but not released by gripper
	//	//AddReward(reward);
	//	//episodeReward += reward;
	//	//if (showDebugText) info += "Placed: " + reward.ToString("F3") + "\n";

	//	///*Grasping cube*/
	//	//reward = (!isPlaced && isGrasped) ? 0.001f : 0f;
	//	//AddReward(reward);
	//	//episodeReward += reward;
	//	//if (showDebugText) info += "Grasping: " + reward.ToString("F3") + "\n";

	//	/*Negative reward for gripper being away from brick */
	//	reward = -0.01f * Vector3.Distance(gripperPosition, brickPosition);
	//	AddReward(reward);
	//	episodeReward += reward;
	//	if (showDebugText) info += "Gripper to brick distance: " + reward.ToString("F3") + "\n";

	//	if (showDebugText) {
	//		actionText += "Rewards:\n";
	//		actionText += "Step R: " + episodeReward.ToString("F3") + "\n";
	//		actionText += info;
	//		actionText += "Cum. R: " + GetCumulativeReward().ToString("F3") + "\n\n";
	//		actionText += "Step: " + StepCount;
	//		//text.text += "Rewards:\n";
	//		//text.text += "Episode R: " + episodeReward.ToString("F3") + "\n";
	//		//text.text += info;
	//		//text.text += "Cum. R: " + GetCumulativeReward().ToString("F3") + "\n\n";
	//		//text.text += "Step: " + StepCount;
	//	}
	//}

	//public override void Heuristic(in ActionBuffers actionsOut) {

	//	var discreteActionsOut = actionsOut.DiscreteActions;
	//	/*Grip*/
	//	discreteActionsOut[0] = RawToDiscreteInput(Input.GetAxisRaw("Gripper"));
	//	/*Position*/
	//	discreteActionsOut[1] = RawToDiscreteInput(Input.GetAxisRaw("X"));
	//	discreteActionsOut[2] = RawToDiscreteInput(Input.GetAxisRaw("Y"));
	//	discreteActionsOut[3] = RawToDiscreteInput(Input.GetAxisRaw("Z"));
    //    /*Rotation*/
	//	//discreteActionsOut[4] = RawToDiscreteInput(Input.GetAxisRaw("YRot"));
	//	discreteActionsOut[4] = RawToDiscreteInput(Input.GetAxisRaw("XRot"));
	//	discreteActionsOut[5] = RawToDiscreteInput(Input.GetAxisRaw("YRot"));
	//	discreteActionsOut[6] = RawToDiscreteInput(Input.GetAxisRaw("ZRot"));

	//}

	//public override void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
	//{
	//	actionMask.SetActionEnabled(4, 1, false);
	//	actionMask.SetActionEnabled(4, 2, false);
	//	actionMask.SetActionEnabled(6, 1, false);
	//	actionMask.SetActionEnabled(6, 2, false);


	//	float brick2target_pos = Vector3.Distance(currentBrick.TargetPosition, currentBrick.transform.position);
	//	/*Only enforcing mask in the first curriculum step*/
	//	if(resetState && STACK_STATE == 1 && brick2target_pos > 0.1f){
	//		actionMask.SetActionEnabled(0, 2, false);
	//		//Debug.Log("masked");
	//	}
	//}

	//private void FixedUpdate() {
    //    /*Manual Control*/
	//	if (resetState) {
	//		if (Input.GetKeyDown("p")) { //Reset state
	//			Debug.Log("p pressed");
	//			EndEpisode();
	//		}
	//	}
	//}

	//static float RawToAbsInput(float input) {
	//	if (input > 0) {
	//		return 1;
	//	} else if (input < 0) {
	//		return -1;
	//	} else {
	//		return 0;
	//	}
	//}
	//static public int RawToDiscreteInput(float input) {
	//	if (input > 0) {
	//		return 2;
	//	} else if (input < 0) {
	//		return 1;
	//	} else {
	//		return 0;
	//	}
	//}
	////static public int RawToDiscreteInput(float input) {
	////	if (input > 0) {
	////		return 2;
	////	} else if (input < 0) {
	////		return 0;
	////	} else {
	////		return 1;
	////	}
	////}
	//static public int DiscreteToMoveInput(float input) {
	//	if (input == 0) {
	//		return 0;
	//	} else if (input == 1) {
	//		return -1;
	//	} else {
	//		return 1;
	//	}
	//}
	////static public int DiscreteToMoveInput(float input) {
	////	if (input == 1) {
	////		return 0;
	////	} else if (input == 0) {
	////		return -1;
	////	} else {
	////		return 1;
	////	}
	////}
	
	
	public override void OnEpisodeBegin()
	{
		decisionRequester.grasped = true;
		if(counter ==1)
		{
			sr.Close();
			Debug.Break();
		}
		counter++;

		var drive = gripper.xPrismaticAB.xDrive;
		float value = 0f;
		drive.target = value;
		gripper.xPrismaticAB.xDrive = drive;

		// force position
        ArticulationReducedSpace newPosition = new ArticulationReducedSpace(value);
        gripper.xPrismaticAB.jointPosition = newPosition;

        // force velocity to zero
        ArticulationReducedSpace newVelocity = new ArticulationReducedSpace(0.0f);
        //gripper.xPrismaticAB.jointVelocity = newVelocity;
		//Debug.Break();
	}

	public override void CollectObservations(VectorSensor sensor)
	{
		//var drive = gripper.xPrismaticAB.xDrive;
		//drive.target = 0f;
		//gripper.xPrismaticAB.xDrive = drive;
	}
	public override void OnActionReceived(ActionBuffers actions)
	{
		string data = StepCount+","+gripper.xPrismaticAB.jointPosition[0]+","+gripper.xPrismaticAB.jointVelocity[0]+","+gripper.xPrismaticAB.jointAcceleration[0];
		sr.WriteLine(data);

		var drive = gripper.xPrismaticAB.xDrive;
		float value = 1f;
		drive.target = value;
		gripper.xPrismaticAB.xDrive = drive;
	}

}
