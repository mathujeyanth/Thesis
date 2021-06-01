using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripControl : MonoBehaviour
{
    public ArticulationBody gripper;
    public ArticulationBody RightFingerAB, LeftFingerAB;
	gripperFingerCollider RightFingerC, LeftFingerC;
	public float maxGrip {get { return 0.25f;}}
    public float gripSpeed {get { return 0.5f;}}

    public Vector3 velocity {get {return gripper.velocity;}}
    public Vector3 velocityNormalized {get {return gripper.velocity / (Time.fixedDeltaTime/Time.maximumDeltaTime);}}
    public Vector3 angularVelocity {get {return gripper.angularVelocity;}}
    public Vector3 angularVelocityNormalized {get {return (gripper.angularVelocity*Mathf.Rad2Deg)/ (Time.fixedDeltaTime/Time.maximumDeltaTime);}}
    
    protected virtual void Start()
    {
        RightFingerC = RightFingerAB.gameObject.GetComponent<gripperFingerCollider>();
		LeftFingerC = LeftFingerAB.gameObject.GetComponent<gripperFingerCollider>();
    }
    public void resetGraspCollider()
    {
        RightFingerC.isGrasped = false;
        RightFingerC.inCollision = false;
        LeftFingerC.isGrasped = false;
        LeftFingerC.inCollision = false;
    }
    public bool isGrasping()
    {
        return RightFingerC.isGrasped && LeftFingerC.isGrasped;
    }
    public bool inCollision(){
        return RightFingerC.inCollision || LeftFingerC.inCollision;
    }
    public float getGrip(){
        return 0.5f*(Mathf.Abs(RightFingerAB.jointPosition[0]) + Mathf.Abs(LeftFingerAB.jointPosition[0])) / maxGrip;
    }
	public void setGrip(float value) {
		/*close:0.25f, open:0f*/
		var rightFingerDrive = RightFingerAB.xDrive;
		var leftFingerDrive = LeftFingerAB.xDrive;
		rightFingerDrive.target = Mathf.Clamp(value,0f,maxGrip);
		leftFingerDrive.target = Mathf.Clamp(value,0f,maxGrip);
		if(RightFingerAB.xDrive.target != rightFingerDrive.target) RightFingerAB.xDrive = rightFingerDrive;
		if(LeftFingerAB.xDrive.target != leftFingerDrive.target) LeftFingerAB.xDrive = leftFingerDrive;
	}
	public void moveGrip(float direction) {
		/*close:0.25f, open:0f*/
		float rightFingerPosition = RightFingerAB.xDrive.target;//RightFingerAB.jointPosition[0];
		float leftFingerPosition = LeftFingerAB.xDrive.target;//LeftFingerAB.jointPosition[0];

		//var rightFingerDrive = RightFingerAB.xDrive;
		//var leftFingerDrive = LeftFingerAB.xDrive;

		//rightFingerDrive.target = rightFingerPosition + -direction * Time.fixedDeltaTime * linearSpeed;
		//leftFingerDrive.target = leftFingerPosition + -direction * Time.fixedDeltaTime * linearSpeed;
		//RightFingerAB.xDrive = rightFingerDrive;
		//LeftFingerAB.xDrive = leftFingerDrive;
		float gripValue = 0.5f*(rightFingerPosition+leftFingerPosition) -direction * Time.fixedDeltaTime * gripSpeed;
		setGrip(gripValue);
	}
}
