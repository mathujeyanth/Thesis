using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
//using System;

using Unity.MLAgents;
//using Unity.Barracuda;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class GripperAgent : Agent {
	/*Gripper*/
	TCPUtility TCP;
	Vector3 TCPStartingPosition;
	public GameObject gripper;
	GripperController gripperController;
	ArticulationBody gripperBody;
	public GameObject BrickContainer;
	BrickManager brickManager;
	ObjectBasic currentBrick;

	public GameObject placeArea;
	Vector3 placeAreaBounds;
	/*Agent*/
	EnvironmentParameters m_ResetParams;

	/*Debug*/
	public bool showDebugText = false;
	public UnityEngine.UI.Text text;
	//public bool showCamera = false;
	public UnityEngine.UI.RawImage camera;
	Texture2D texture;
	Unity.MLAgents.Sensors.CameraSensorComponent WristCamera;

	public Transform TCPStart;
	Vector3 GripperStart;
	public Transform GripperCenter;
	int STACK_STATE;
	float[] BRICK_ROTAIONS = { 0f, 90f, 180f, 270f };

	string observationText = "";
	string actionText = "";
	bool graspedCube=false;

	public override void Initialize() {
		TCP = gameObject.GetComponent<TCPUtility>();
		gripperController = gripper.GetComponent<GripperController>();
		gripperBody = gripper.GetComponent<ArticulationBody>();
		brickManager = BrickContainer.GetComponent<BrickManager>();

		placeAreaBounds = placeArea.GetComponent<Collider>().bounds.extents;

		m_ResetParams = Academy.Instance.EnvironmentParameters;
		//Academy.Instance.AutomaticSteppingEnabled = false;

	}
	//private void FixedUpdate() {
	//	if (Academy.IsInitialized && graspedCube)
	//	{
	//		MakeRequests(StepCount);
	//		//RequestDecision();
	//	}
	//	//Academy.Instance.EnvironmentStep();
	//}
	////private void Awake() {
	////	Academy.Instance.AgentPreStep += MakeRequests;
	////}
	//void MakeRequests(int academyStepCount){
	//	if( academyStepCount % 5 ==0) RequestDecision();
	//	else RequestAction();
	//}

	private void Start() {
		//if (showCamera) {
		//	WristCamera = gameObject.GetComponent<Unity.MLAgents.Sensors.CameraSensorComponent>();
		//	texture = new Texture2D(WristCamera.Width, WristCamera.Height);
		//	var image = WristCamera.CreateSensor().GetCompressedObservation();
		//	System.IO.File.WriteAllBytes("image.png", image);
		//	camera.color = new UnityEngine.Color(1, 1, 1, 1);
		//}
	}
	public override void OnEpisodeBegin() {
		graspedCube=false;
		STACK_STATE = (int)m_ResetParams.GetWithDefault("N_BRICKS", 3);
		int N_BRICKS = STACK_STATE;
		//Debug.Log(STACK_STATE);
		if (STACK_STATE == 0) {
			if (currentBrick == null) {
				brickManager.AddBrick((int)BrickType.B222, (int)BrickColor.random, false);
				Vector3 position = brickManager.pickArea.transform.position;
				brickManager.bricksInstantiated.Last().transform.position = position + Vector3.up * 0.2f;
			}
			ObjectBasic tmpBrick = brickManager.bricksInstantiated.Last();
			tmpBrick.TargetPosition = placeArea.transform.position;
			tmpBrick.TargetPosition.y += 0.5f*tmpBrick.height;//tmpBrick.gameObject.GetComponent<Renderer>().bounds.size.y / 2f;//placeAreaExtents.y;
			tmpBrick.TargetPosition.y += 0.1f * Random.Range(0, 10);
			tmpBrick.TargetPosition.x += 0.1f * Random.Range(-4, 4);
			tmpBrick.TargetPosition.z += 0.1f * Random.Range(-4, 4);

			float brick_rotation = BRICK_ROTAIONS[Random.Range(0, 3)];
			tmpBrick.TargetRotation = Quaternion.Euler(0f, brick_rotation, 0f);
		}
		/*Instantiate new bricks*/
		else if (currentBrick == null || (currentBrick != null && !currentBrick.IsCompleted) || brickManager.placedAllBricks() || brickManager.placedAndRemoved()) { // 
			brickManager.DestroyAllBricks(); // Destroy old

			Vector3 position = brickManager.pickArea.transform.position;// + Vector3.up * 0.05f;
			for (int i = 0; i < N_BRICKS; i++) {
				brickManager.AddBrick((int)BrickType.B222, (int)BrickColor.random, false);
				ObjectBasic tmpBrick = brickManager.bricksInstantiated.Last();
				tmpBrick.transform.position = position + Vector3.up * 0.2f;
				/*Random position on placearea for first brick*/
				if (i == 0) {
					tmpBrick.TargetPosition = placeArea.transform.position;

					tmpBrick.TargetPosition.y += tmpBrick.height;//0.045f;//TODO: find floor 2 brick height. Currently a static value 
					tmpBrick.TargetPosition.x += 0.08f * Random.Range(-4, 4); //[-0.8f,0.8f]
					tmpBrick.TargetPosition.z += 0.08f * Random.Range(-4, 4);
				}
				/*Place other bricks on top of the previous*/
				else {
					ObjectBasic previousBrick = brickManager.bricksInstantiated[brickManager.bricksInstantiated.Count - 2];
					tmpBrick.TargetPosition = previousBrick.TargetPosition;
					tmpBrick.TargetPosition.y += 0.1f;//TODO: find brick 2 brick height. Currently a static value
				}
				float brick_rotation = BRICK_ROTAIONS[Random.Range(0, 3)];
				tmpBrick.TargetRotation = Quaternion.Euler(0f, 0f*brick_rotation, 0f);
				//tmpBrick.TargetRotation = Quaternion.identity;
				position = tmpBrick.transform.position;
			}
			/*TODO: Add bricks based on a list and set their targetposition*/
			currentBrick = null; //No previous brick
		}
		/*Initialize brick to be placed*/
		currentBrick = brickManager.GetCurrentBrick();
		currentBrick.resetTag();

		/*Testing for single brick*/
		//currentBrick.TargetRotation = Quaternion.identity;
		//currentBrick.TargetPosition = TCPStart.InverseTransformPoint(placeArea.transform.position);
		//currentBrick.TargetPosition.y += currentBrick.gameObject.GetComponent<Renderer>().bounds.size.y/2f;
		//currentBrick.TargetPosition.y += 0.2f * Random.Range(0,2);
		//currentBrick.TargetPosition.x += 0.2f * Random.Range(-2, 2);
		//currentBrick.TargetPosition.z += 0.2f * Random.Range(-2, 2);

		/*Reset gripper configuration*/
		gripperController.resetCollision();
		TCP.forceReset();
		gripperController.forceResetGrip();
		TCP.GetComponent<ArticulationBody>().SetJointPositions((new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>());
		TCP.GetComponent<ArticulationBody>().SetJointVelocities((new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>());
		TCP.GetComponent<ArticulationBody>().SetJointAccelerations((new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>());
		////GripperStart=GripperCenter.position;
		/*Place brick between center of gripper fingers*/
		if (STACK_STATE != 0) {
			//Vector3 gripperCenter = gripperController.GetGripperCenterPosition();
			currentBrick.transform.position = GripperCenter.position;
			//currentBrick.transform.position = gripperCenter;
			currentBrick.transform.rotation = Quaternion.identity;
			currentBrick.calculateDistancePosition(yMultiplier: 1f);

			/*Close gripper finger*/
			//StartCoroutine(MyCoroutine());
			//StartCoroutine(ResetHandCoroutine());
			gripperController.setGrip(0.7f); //alternatively calculate object size and use that to set the grip Vector3 objectSize = Vector3.Scale(transform.localScale, GetComponent().mesh.bounds.size);

			//float[] gripJointPosition =new float[] { 0, 0, 0, 0, 0, 0, 0,0};
			//gripJointPosition[gripperController.fingerIndices[0]]=-0.17f;
			//gripJointPosition[gripperController.fingerIndices[1]]=0.17f;
			//TCP.GetComponent<ArticulationBody>().SetJointPositions((gripJointPosition).ToList<float>());
			//TCP.GetComponent<ArticulationBody>().SetJointVelocities((new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>());
			//TCP.GetComponent<ArticulationBody>().SetJointAccelerations((new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>());


			//List<float> some =(new float[] { 0, 0, 0, 0, 0, 0, 0, 0 }).ToList<float>();
			//TCP.GetComponent<ArticulationBody>().GetJointPositions(some);
			//Debug.Log(some[6]+" "+some[7]);
			currentBrick.rb.useGravity=true;
		}
		//Debug.Log("first: "+gripperController.getGrip());
		//Debug.Log("main:"+gripperController.graspingCube());
	}

	IEnumerator ResetHandCoroutine()
    {
		//Debug.Log($"1: {gripperController.getGrip()}");
		TCP.forceReset();
		//gripperController.forceResetGrip();
		gripperController.setGrip(0f);
		Vector3 TCPpos = TCP.getPosition();
        while (!(TCPpos == Vector3.zero))
        {
			TCPpos = TCP.getPosition();
			gripperController.gripState = HandState.Neg;
            yield return null;
        }
		yield return null;
		//Debug.Log($"2: {gripperController.getGrip()}");
		currentBrick.rb.useGravity=false;
		currentBrick.rb.position = GripperCenter.position;
		currentBrick.rb.rotation = Quaternion.identity;
		yield return null;
		currentBrick.calculateDistancePosition(yMultiplier: 1f);
		//yield return new WaitForSeconds(Time.fixedDeltaTime);
        yield return true;
    }
    IEnumerator GraspCubeCoroutine()
    {
		//Debug.Log("3");
		//gripperController.setGrip(0.7f);
		TCP.gripState = HandState.Pos;
        while(!gripperController.graspingCube()){
			//gripperController.setGrip(0.7f);
            //Debug.Log(pincherController.grip);
			yield return null;
        }
		yield return null;
		//Debug.Log($"4: {gripperController.getGrip()}");
		currentBrick.rb.useGravity=true;
        graspedCube=true;
        yield return true;
    }

	public override void CollectObservations(VectorSensor sensor) {
		float grip = gripperController.getGrip() / gripperController.gripMaxLimit;
		//Vector3 gripperCenter = gripperController.GetGripperCenterPosition();

		//Vector3 TCPposition = TCP.getPosition();
		//Quaternion TCProtation = Quaternion.Euler(TCP.getRotation());
		Vector3 TCPlinearVelocity = gripperBody.velocity;
		Quaternion TCPangularVelocity = Quaternion.Euler(gripperBody.angularVelocity);

		Vector3 gripperPosition = TCPStart.InverseTransformPoint(GripperCenter.position);
		Quaternion gripperRotation = GripperCenter.rotation;

		Vector3 targetPosition = TCPStart.InverseTransformPoint(currentBrick.TargetPosition);
		Quaternion targetRotation = currentBrick.TargetRotation;

		string info = "Observations:\n";
		Vector3 vec3;
		Quaternion quat4;

		/*Gripper: 1*/
		float f1 = grip;
		if (showDebugText) info += "Grip: " + f1.ToString("F3") + "\n";
		sensor.AddObservation(f1);

		/*TCP and target distance: 7*/
		vec3 = (targetPosition - gripperPosition)*100;
		quat4 = targetRotation.ShortestRotation(gripperRotation);
		sensor.AddObservation(vec3);
		sensor.AddObservation(quat4);
		if (showDebugText) info += "TCP: " + vec3.ToString("F3") + " | " + quat4.eulerAngles.EulerLimit().ToString("F3") + "\n";

		/*TCP Velocity: 7*/
		sensor.AddObservation(TCPlinearVelocity / (TCP.positionSpeed * Time.fixedDeltaTime * 10f)); //TCP.positionSpeed should be the max speed but for some reason this part is
		sensor.AddObservation(TCPangularVelocity);

		/*Forces: 6*/
		//Add forces measured by tcp (and gripper fingers if possible)

		/*Brick and TCP distance: 7*/
		Vector3 brickPosition = TCPStart.InverseTransformPoint(currentBrick.transform.position);
		Quaternion brickRotation = currentBrick.transform.rotation;

		//vec3 = gripperPosition-brickPosition;
		//quat4 = targetRotation.ShortestRotation(brickRotation); 
		//sensor.AddObservation(vec3);
		//sensor.AddObservation(quat4);
		//if (showDebugText) info+="G2B: "+vec3.ToString("F3")+" | "+quat4.eulerAngles.EulerLimit().ToString("F3")+"\n";

		///*Brick and target distance: 7*/
		//vec3 = targetPosition - brickPosition;
		//quat4 = brickRotation.ShortestRotation(currentBrick.TargetRotation);
		//sensor.AddObservation(vec3);
		//sensor.AddObservation(quat4);
		//if (showDebugText) info+="B2T: "+vec3.ToString("F3")+" | "+quat4.eulerAngles.EulerLimit().ToString("F3")+"\n";

		if (showDebugText) {
			observationText = "Curriculum State: " + STACK_STATE + "\n";
			observationText += info + "\n";
			observationText += "Desired: " + targetPosition.ToString("F3") + " | " + targetRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
			observationText += "Brick: " + brickPosition.ToString("F3") + " | " + brickRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
			observationText += "Gripper: " + gripperPosition.ToString("F3") + " | " + gripperRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
			//text.text = "Curriculum State: " + STACK_STATE + "\n";
			//text.text += info + "\n";
			//text.text += "Desired: " + targetPosition.ToString("F3") + " | " + targetRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
			//text.text += "Brick: " + brickPosition.ToString("F3") + " | " + brickRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
			//text.text += "Gripper: " + gripperPosition.ToString("F3") + " | " + gripperRotation.eulerAngles.EulerLimit().ToString("F3") + "\n";
		}
		//if (showCamera) {
		//	var image = WristCamera.CreateSensor().GetCompressedObservation();
		//	texture.LoadImage(image);
		//	camera.texture = texture;
		//}

	}

	//public override void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
	//{
	//	float brick2target_pos = Vector3.Distance(currentBrick.TargetPosition, currentBrick.transform.position);
	//	bool isGrasped = gripperController.graspingCube();
	//	////if(StepCount<10){
	//	//if(!graspedCube || StepCount<50 || brick2target_pos > 0.1f){
	//	//if(!graspedCube){// || brick2target_pos > 0.1f){
	//	//	actionMask.WriteMask(6,new int[1]{0});
	//	//}
	//	if( brick2target_pos > 0.1f){
	//		actionMask.WriteMask(6,new int[1]{0});
	//	}
	//}
	/*Stack randomly in a range between 5 to 10 bricks on top of each other*/
	public override void OnActionReceived(ActionBuffers actions) {
		int stateSum = 0;
		HandState[] actionsUsed = new HandState[7];
		for (int i = 0; i < 7; i++) {
			/*Rotation around x and z is disabled*/
			actionsUsed[i] = (i == 4 || i == 6) ? HandState.Fixed : gripperController.StateForInput(actions.DiscreteActions[i] - 1f); // -1f to go from one hot to -1f:1f;
			stateSum += actionsUsed[i] == HandState.Fixed ? 0 : 1;
		}
		
		TCP.gripState = actionsUsed[0];
		for (int i = 0; i < 6; i++) {
			TCP.TCPStates[i] = actionsUsed[i+1];
		}

		if (showDebugText) actionText = "\nActions: " + string.Join(",", actionsUsed.Cast<int>()) + "\n\n";

		if (STACK_STATE != 0) {
			AssignRewardsBricks(stateSum);
		} else {
			AssignRewardsMove(stateSum);
		}
		if (showDebugText) text.text = observationText + actionText;
	}
	void AssignRewardsMove(int stateSum) {
		float gripper2target_pos = Vector3.Distance(currentBrick.TargetPosition, GripperCenter.position);
		float gripper2target_rot = Quaternion.Angle(currentBrick.TargetRotation, GripperCenter.transform.rotation) / 180f;

		if (gripper2target_pos < 0.05f && gripper2target_rot < 0.05f) {
			SetReward(1f);
			EndEpisode();
		}

		/*Reward Shaping*/
		string info = "";
		float reward = 0f;
		float episodeReward = 0f;

		reward = -0.001f;
		AddReward(reward);//Time used penalty: small negative
		episodeReward += reward;
		if (showDebugText) info += "Time penalty: " + reward.ToString("F3") + "\n";

		reward = stateSum == 0 ? -0.001f : 0f;  // Penalty for standing still
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "No Action Penalty: " + reward.ToString("F3") + "\n";

		/*Gripper penalty for being away from target*/
		reward = -0.01f * gripper2target_pos;
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "G2T Pos: " + reward.ToString("F4") + "\n";

		reward = -0.01f * gripper2target_rot;
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "G2T Rot: " + reward.ToString("F4") + "\n";


		if (showDebugText) {
			text.text += "Rewards:\n";
			text.text += "Episode R: " + episodeReward.ToString("F3") + "\n";
			text.text += info;
			text.text += "Cum. R: " + GetCumulativeReward().ToString("F3") + "\n\n";
			text.text += "Step: " + StepCount;
		}
	}

	void AssignRewardsBricks(int stateSum) {
		Vector3 brickPosition = currentBrick.transform.position;
		Quaternion brickRotation = currentBrick.transform.rotation;
		Vector3 distanceToPlaceArea = (placeArea.transform.position - brickPosition);
		Vector3 gripperPosition = GripperCenter.position;
		float brick2target_pos = Vector3.Distance(currentBrick.TargetPosition, brickPosition);
		float brick2target_rot = Quaternion.Angle(currentBrick.TargetRotation, brickRotation) / 180f;

		/*If brick is outside XY plane: big negative + end episode*/
		if (Mathf.Abs(distanceToPlaceArea.x) > placeAreaBounds.x || Mathf.Abs(distanceToPlaceArea.z) > placeAreaBounds.z) {
			SetReward(-1f);
			EndEpisode();
		}

		/*If brick and grippercenter distance too big*/
		if (Vector3.Distance(gripperPosition, brickPosition) > 0.5f/*5cm*/) {
			SetReward(-1f);
			EndEpisode();
		}

		/*If moving other bricks: large negative + end?*/
		if (brickManager.placedAndRemoved()){
			//SetReward(-1f);
			EndEpisode();
		}

		/*If brick is stacked: big positive + end episode*/
		bool isPlaced = currentBrick.isPlaced();
		bool isGrasped = gripperController.graspingCube();
		if (isPlaced && !isGrasped && currentBrick.rb.velocity.magnitude < 0.05f) {
			currentBrick.IsCompleted = true;
			SetReward(1f);
			currentBrick.removeTag();
			EndEpisode();
		}

		/*Reward Shaping*/

		string info = "";
		float reward = 0f;
		float episodeReward = 0f;

		reward = -0.001f;
		AddReward(reward);//Time used penalty: small negative
		episodeReward += reward;
		if (showDebugText) info += "Time penalty: " + reward.ToString("F3") + "\n";

		//reward = stateSum == 0 ? -0.001f : 0f;  // Penalty for standing still
		//AddReward(reward);
		//episodeReward += reward;
		//if (showDebugText) info += "No Action Penalty: " + reward.ToString("F3") + "\n";

		bool inCollison = gripperController.inCollision();
		reward = inCollison ? -0.01f : 0f;
		AddReward(reward); // Penalty for collision
		episodeReward += reward;
		if (showDebugText) info += "In collision: " + reward.ToString("F3") + "\n";

		//reward = isGrasped ? 0.001f * (1f - brick2target_pos * 10) : 0f;
		reward = 0.001f * Mathf.Clamp01(1f - brick2target_pos * 10);
		//reward = 0.001f * (1f - brick2target_pos * 10);
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "Brick position distance: " + reward.ToString("F4") + "\n";

		//reward = isGrasped ? 0.001f * (1f - brick2target_rot * 10) : 0f;
		reward = 0.001f * Mathf.Clamp01(1f - brick2target_rot * 10);
		//reward = 0.001f * (1f - brick2target_rot * 10);
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "Brick rotation distance: " + reward.ToString("F4") + "\n";

		//reward = isPlaced ? 0.1f : 0f;  //If placed, but not released by gripper
		//AddReward(reward);
		//episodeReward += reward;
		//if (showDebugText) info += "Placed: " + reward.ToString("F3") + "\n";

		///*Grasping cube*/
		//reward = (!isPlaced && isGrasped) ? 0.001f : 0f;
		//AddReward(reward);
		//episodeReward += reward;
		//if (showDebugText) info += "Grasping: " + reward.ToString("F3") + "\n";

		/*Negative reward for gripper being away from brick */
		reward = -0.01f * Vector3.Distance(gripperPosition, brickPosition);
		AddReward(reward);
		episodeReward += reward;
		if (showDebugText) info += "Distance gripper to brick: " + reward.ToString("F3") + "\n";

		if (showDebugText) {
			actionText += "Rewards:\n";
			actionText += "Episode R: " + episodeReward.ToString("F3") + "\n";
			actionText += info;
			actionText += "Cum. R: " + GetCumulativeReward().ToString("F3") + "\n\n";
			actionText += "Step: " + StepCount;
			//text.text += "Rewards:\n";
			//text.text += "Episode R: " + episodeReward.ToString("F3") + "\n";
			//text.text += info;
			//text.text += "Cum. R: " + GetCumulativeReward().ToString("F3") + "\n\n";
			//text.text += "Step: " + StepCount;
		}
	}


	public override void Heuristic(in ActionBuffers actionsOut) {
		//var continousActionsOut = actionsOut.ContinuousActions;
		///*Position*/
		//continousActionsOut[0] = Input.GetAxis("X");
		//continousActionsOut[1] = Input.GetAxis("Y");
		//continousActionsOut[2] = Input.GetAxis("Z");

		///*Rotation*/
		//continousActionsOut[3] = Input.GetAxis("XRot");
		//continousActionsOut[4] = Input.GetAxis("YRot");
		//continousActionsOut[5] = Input.GetAxis("ZRot");

		///*Grip*/
		//continousActionsOut[6] = Input.GetAxis("Gripper");

		var discreteActionsOut = actionsOut.DiscreteActions;
		/*Grip*/
		discreteActionsOut[0] = HeuristicForInput(Input.GetAxisRaw("Gripper"));
		/*Position*/
		discreteActionsOut[1] = HeuristicForInput(Input.GetAxisRaw("X"));
		discreteActionsOut[2] = HeuristicForInput(Input.GetAxisRaw("Y"));
		discreteActionsOut[3] = HeuristicForInput(Input.GetAxisRaw("Z"));

		/*Rotation*/
		discreteActionsOut[4] = HeuristicForInput(Input.GetAxisRaw("XRot"));
		discreteActionsOut[5] = HeuristicForInput(Input.GetAxisRaw("YRot"));
		discreteActionsOut[6] = HeuristicForInput(Input.GetAxisRaw("ZRot"));

	}
	public int HeuristicForInput(float input) {
		if (input > 0) {
			return 2;
		} else if (input < 0) {
			return 0;
		} else {
			return 1;
		}
	}
}
