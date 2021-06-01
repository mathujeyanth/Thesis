using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TCPUtility : MonoBehaviour {
	public bool useUtility = false;
	GameObject TCP;
	ArticulationBody[] positionArticulationTCP = new ArticulationBody[3];
	ArticulationBody[] rotationArticulationTCP = new ArticulationBody[3];
	Vector3 positionTCP = Vector3.zero;
	Vector3 rotationTCP = Vector3.zero;


	GameObject gripper;
	GripperController gripperController;
	public Vector3 setPosition;
	public Vector3 setRotation; //should have been quaternion
	public float positionSpeed = 0.5f;
	public float positionMax = 1f;
	public float rotationSpeed = 180f;
	public float rotationMax = 360f;
	public float grip;
	public HandState[] TCPStates;
	public HandState gripState;

	Vector3 startPosition;
	Vector3 startRotation;
	public float[] lastPositionVelocity = new float[3];
	public HandState[] lastHandstates = new HandState[3];
	void Awake() {
		var positonTCPnames = new string[] { "xPrismatic", "yPrismatic", "zPrismatic" };
		var rotationTCPnames = new string[] { "xRevolute", "yRevolute", "zRevolute" };
		TCP = gameObject;
		string articulationName = "";
		for (int i = 0; i < positonTCPnames.Length; i++) {
			articulationName += positonTCPnames[i];
			positionArticulationTCP[i] = TCP.transform.Find(articulationName).GetComponent<ArticulationBody>();
			startPosition[i] = positionArticulationTCP[i].transform.position[i];
			//positionArticulationTCP[i].maxJointVelocity=1f;
			articulationName += "/";
		}
		for (int i = 0; i < rotationTCPnames.Length; i++) {
			articulationName += rotationTCPnames[i];
			rotationArticulationTCP[i] = TCP.transform.Find(articulationName).GetComponent<ArticulationBody>();
			startRotation[i] = rotationArticulationTCP[i].transform.eulerAngles[i];
			//rotationArticulationTCP[i].maxJointVelocity=rotationMax*Mathf.Deg2Rad;
			articulationName += "/";
		}

		gripper = TCP.transform.Find(articulationName + "HandE").gameObject;
		gripperController = gripper.GetComponent<GripperController>();

		TCPStates = new HandState[6];
		for (int i = 0; i < TCPStates.Length; i++) {
			TCPStates[i] = HandState.Fixed;
		}
		gripState = HandState.Fixed;

	}

	void FixedUpdate() {
		UpdatePositionTCP();
		UpdateRotationTCP();
		gripperController.gripState = gripState;
	}
	public void forceReset() {

		for (int i = 0; i < positionArticulationTCP.Length; i++) {
			Vector3 tmpPosition = positionArticulationTCP[i].gameObject.transform.position;
			tmpPosition[i] = startPosition[i];
			positionArticulationTCP[i].gameObject.transform.position = tmpPosition;

			var tmpDrive = positionArticulationTCP[i].xDrive;
			tmpDrive.target = 0f;
			positionArticulationTCP[i].xDrive = tmpDrive;
		}
		for (int i = 0; i < rotationArticulationTCP.Length; i++) {
			Vector3 tmpRotation = rotationArticulationTCP[i].gameObject.transform.eulerAngles;
			tmpRotation[i] = startRotation[i];
			rotationArticulationTCP[i].gameObject.transform.eulerAngles = tmpRotation;

			var tmpDrive = rotationArticulationTCP[i].xDrive;
			tmpDrive.target = 0f;
			rotationArticulationTCP[i].xDrive = tmpDrive;
		}
		for (int i = 0; i < TCPStates.Length; i++) {
			TCPStates[i] = HandState.Fixed;
		}
	}
	public Vector3 getPosition() {
		Vector3 newPosition = Vector3.zero;
		for (int i = 0; i < positionArticulationTCP.Length; i++) {
			newPosition[i] = positionArticulationTCP[i].jointPosition[0];
		}
		return newPosition;
	}
	public Vector3 getRotation() {
		Vector3 newRotation = Vector3.zero;
		for (int i = 0; i < rotationArticulationTCP.Length; i++) {
			newRotation[i] = rotationArticulationTCP[i].jointPosition[0] * Mathf.Rad2Deg;
		}
		return newRotation;
	}

	public void setPositionTCP(Vector3 position) {
		for (int i = 0; i < positionArticulationTCP.Length; i++) {
			var tmpVelocity = positionArticulationTCP[i].jointVelocity;
			tmpVelocity[0] = 0f;

			var tmpPosition = positionArticulationTCP[i].jointPosition;
			tmpPosition[0] = positionMax * Mathf.Clamp(position[i], -1f, 1f);

			var tmpDrive = positionArticulationTCP[i].xDrive;
			tmpDrive.target = positionMax * Mathf.Clamp(position[i], -1f, 1f);

			positionArticulationTCP[i].xDrive = tmpDrive;
			positionArticulationTCP[i].jointPosition = tmpPosition;
			positionArticulationTCP[i].jointVelocity = tmpVelocity;
		}
	}
	public void setRotationTCP(Vector3 rotation) {
		for (int i = 0; i < rotationArticulationTCP.Length; i++) {
			var tmpVelocity = rotationArticulationTCP[i].jointVelocity;
			tmpVelocity[0] = 0f;

			var tmpPosition = rotationArticulationTCP[i].jointPosition;
			tmpPosition[0] = rotationMax * Mathf.Clamp(rotation[i], -1f, 1f);

			var tmpDrive = rotationArticulationTCP[i].xDrive;
			tmpDrive.target = rotationMax * Mathf.Clamp(rotation[i], -1f, 1f);

			rotationArticulationTCP[i].xDrive = tmpDrive;
			rotationArticulationTCP[i].jointPosition = tmpPosition;
			rotationArticulationTCP[i].jointVelocity = tmpVelocity;
		}
	}
	public void UpdatePositionTCP() {
		Vector3 currentPosition = getPosition();
		for (int i = 0; i < positionArticulationTCP.Length; i++) {
			if (TCPStates[i] != HandState.Fixed) {
				var tempDrive = positionArticulationTCP[i].xDrive;
				float targetPosition = currentPosition[i] + Mathf.Clamp((float)TCPStates[i] * positionSpeed,-1f,1f) * Time.fixedDeltaTime;
				tempDrive.target = Mathf.Clamp(targetPosition, -positionMax, positionMax);
				positionArticulationTCP[i].xDrive = tempDrive;
			}
			//lastPositionVelocity[i] = positionArticulationTCP[i].jointVelocity[0];
			
			//var tempDrive = positionArticulationTCP[i].xDrive;
			//float jointVelocity = 0f;
			//float maxAcceleration = 1f;
			//if(TCPStates[i] !=HandState.Fixed){
			//	jointVelocity=(float)TCPStates[i]*maxAcceleration*Time.fixedDeltaTime;
			//	tempDrive.targetVelocity = Mathf.Clamp(positionArticulationTCP[i].jointVelocity[0]+jointVelocity, -1f, 1f);
			//	positionArticulationTCP[i].xDrive = tempDrive;
			//}else
			//{
			//	tempDrive.targetVelocity = 0f;
			//	positionArticulationTCP[i].xDrive = tempDrive;
			//}
			//positionArticulationTCP[i].jointVelocity = new ArticulationReducedSpace((float)TCPStates[i]);

			//positionArticulationTCP[i].jointAcceleration = new ArticulationReducedSpace((float)TCPStates[i]);


			//float jointPosition = 0f;
			//float jointVelocity = lastPositionVelocity[i];
			//float maxAcceleration = 1f;
			//float jointAcceleration = (float)TCPStates[i]*maxAcceleration;
			//float newVelocity = 0f;
			//if(TCPStates[i] != HandState.Fixed){
			//	float deltaVelocity = 0.5f*jointAcceleration*Time.fixedDeltaTime;
			//	newVelocity = jointVelocity + deltaVelocity;
			//	if(newVelocity < 1f || newVelocity > -1f){
			//		jointPosition = currentPosition[i]+newVelocity*Time.fixedDeltaTime;
			//	}
			//	//jointPosition = currentPosition[i]+jointAcceleration*(Time.fixedDeltaTime*Time.fixedDeltaTime);
			//	//lastPositionVelocity[i] = jointAcceleration*Time.fixedDeltaTime;
			//}
			//else
			//{
			//	float deltaVelocity = - Mathf.Sign(jointVelocity)*(jointVelocity*jointVelocity)/(2f*maxAcceleration);
			//	newVelocity = jointVelocity + deltaVelocity;
			//	if(newVelocity>0.00001f || newVelocity<-0.00001f){
			//		jointPosition = currentPosition[i] + newVelocity*Time.fixedDeltaTime;
			//	}
			//}
			//if(i==1){
			//	Debug.Log(jointPosition);
			//}	


			//var tempDrive = positionArticulationTCP[i].xDrive;
			//float targetPosition = jointPosition;
			//tempDrive.target = Mathf.Clamp(targetPosition, -positionMax, positionMax);
			//positionArticulationTCP[i].xDrive = tempDrive;
			//lastPositionVelocity[i] = Mathf.Clamp(newVelocity,-1f,1f);
		}
	}
	public void UpdateRotationTCP() {
		Vector3 currentRotation = getRotation();
		for (int i = 0; i < rotationArticulationTCP.Length; i++) {
			if (TCPStates[3 + i] != HandState.Fixed) {
				var tempDrive = rotationArticulationTCP[i].xDrive;
				float targetRotation = currentRotation[i] + ((float)TCPStates[3 + i] * rotationSpeed) * Time.fixedDeltaTime;
				tempDrive.target = Mathf.Clamp(targetRotation, -rotationMax, rotationMax);
				rotationArticulationTCP[i].xDrive = tempDrive;
			}
		}
	}
}
