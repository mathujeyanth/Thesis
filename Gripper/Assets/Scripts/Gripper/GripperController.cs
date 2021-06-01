using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum HandState { Fixed = 0, Neg = -1, Pos = 1 };
public enum collisionStatus { brick = 0, other =1};

public class GripperController : MonoBehaviour
{
    /*Gripper*/
    GameObject gripper;

    // Grip - the extent to which the pincher is closed. 0: fully open, 1: fully closed.
    public float gripMaxLimit = 0.25f;
    public float gripSpeed = 1.0f;
    public HandState gripState = HandState.Fixed;

    /*Fingers*/
    static int numOfFingers=2;
    ArticulationBody[] fingers = new ArticulationBody[numOfFingers];
    Vector3 TCPposition;
    fingerCollider[] fingersCollider = new fingerCollider[numOfFingers];
    float[] multipliers = new float[]{-1,1};
    Vector3[] openPosition = new Vector3[numOfFingers];

    public int[] fingerIndices = new int[2];


    void Awake() {
        gripper = this.gameObject;
        var fingerNames = new string[]{"FingerA","FingerB"};
        for (int i = 0; i < fingerNames.Length; i++){
            fingers[i] = gripper.transform.Find(fingerNames[i]).GetComponent<ArticulationBody>();
            openPosition[i] = fingers[i].gameObject.transform.position;
            //fingers[i].maxJointVelocity=gripSpeed;
            fingersCollider[i] = fingers[i].gameObject.GetComponent<fingerCollider>();

            fingerIndices[i]=fingers[i].index-2;
        }
    }
    public void resetCollision(){
        fingersCollider[0].touchedBrick=false;    
        fingersCollider[1].touchedBrick=false; 
        fingersCollider[0].inCollision=false;    
        fingersCollider[1].inCollision=false; 
    }
    public void forceResetGrip(){
        for (int i = 0; i < fingers.Length; i++)
        {
            fingers[i].gameObject.transform.position = openPosition[i];

            var tmpDrive = fingers[i].zDrive;
			tmpDrive.target = 0f;
			fingers[i].zDrive = tmpDrive;
        }
        gripState = HandState.Fixed;
    }
    public void setGrip(float grip) {
		for (int i = 0; i < fingers.Length; i++) {
            //fingers[i].gameObject.transform.position = openPosition[i]+multipliers[i] *Vector3.back;

            var tmpDrive = fingers[i].zDrive;
			tmpDrive.target = multipliers[i] * gripMaxLimit*Mathf.Clamp01(grip);

			fingers[i].zDrive = tmpDrive;
            fingers[i].jointPosition=new ArticulationReducedSpace(tmpDrive.target);
            fingers[i].jointVelocity=new ArticulationReducedSpace(0.0f);
		}
        //gripState = HandState.Fixed;
    }
    public void UpdateGrip() {
        if (gripState != HandState.Fixed) {
            for (int i = 0; i < fingers.Length; i++) {
                var tmpDrive = fingers[i].zDrive;
                float gripChange = multipliers[i] *((float)gripState*gripSpeed)*Time.fixedDeltaTime;
                float targetGrip = multipliers[i] *getGrip() + gripChange;

                if(i==0) tmpDrive.target = Mathf.Clamp(targetGrip, gripMaxLimit*multipliers[i],0);
                if(i==1) tmpDrive.target = Mathf.Clamp(targetGrip,0, gripMaxLimit*multipliers[i]);
                fingers[i].zDrive = tmpDrive;
            }
        }
    }
    public Vector3 GetGripperCenterPosition(float tipMultiplier=0.3f){
        return 0.5f*(fingers[0].transform.position + fingers[1].transform.position)  + Vector3.down * tipMultiplier; //vector down to get the gripper tip position
    }
    public Vector3 GetGripperCenterRotation(){
        return 0.5f*(fingers[0].transform.rotation.eulerAngles + fingers[1].transform.rotation.eulerAngles); //Not sure if this works properly
    }
    public float getGrip(){
        return 0.5f*(Mathf.Abs(fingers[0].jointPosition[0]) + Mathf.Abs(fingers[1].jointPosition[0]));
    }
    public bool graspingCube(){
        return fingersCollider[0].touchedBrick && fingersCollider[1].touchedBrick;
    }
    public bool inCollision(){
        return fingersCollider[0].inCollision || fingersCollider[1].inCollision;
    }
    void FixedUpdate() {
        UpdateGrip();
    }
    public HandState StateForInput(float input) {
        if (input == 1f) {
            return HandState.Pos;
        } else if (input == -1f) {
            return HandState.Neg;
        } else {
            return HandState.Fixed;
        }
    }

}
