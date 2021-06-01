using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using System.IO;

public class EETest : MonoBehaviour
{
	enum joints
	{
		g,xPris,yPris,zPris,xRev,yRev,zRev
	}
    TCPControl gripper;
    [ReadOnly,SerializeField] bool resetState = false;
	public bool DirectControl=false;
	bool startMonitoring = false;
	StreamWriter sr;
	string fileName = "xPrismaticAdjusted.txt";
	int testJoint = (int)joints.xPris;
	float maxJointLimit = 360;
	int testRuns = 3;
	float targetPosition = 0.1f;//180*Mathf.Deg2Rad;
	

    private void Start() {
        gripper = GetComponent<TCPControl>();
        if (File.Exists(fileName))
        {
            Debug.Log(fileName+" already exists.");
        }
		
        gripper.setTCP(Vector3.zero,Vector3.up);
        gripper.setGrip(0f);

		ArticulationBody testAB = gripper.GetABs()[testJoint-1];
		var tmpDrive=testAB.xDrive;
		tmpDrive.lowerLimit=-maxJointLimit;
		tmpDrive.upperLimit=maxJointLimit;
		testAB.xDrive = tmpDrive;

        //StartCoroutine(ResetGripperCheck());
		if(!DirectControl){
			var file = File.Open(fileName,FileMode.OpenOrCreate,FileAccess.ReadWrite);
			sr = new StreamWriter(file);
			sr.WriteLine("time,pos,vel,acc");
			StartCoroutine(RepeatMovement());
		}
    }
	IEnumerator ResetGripperCheck(){
		/*Check if gripper is reset (standing still)*/
		while (!gripper.isGripperSleeping())
		{
			//Debug.Log("begun");
			yield return new WaitForFixedUpdate();
		}
		yield return new WaitForFixedUpdate();
        resetState = true;
	}

    private void FixedUpdate() {
		if(DirectControl)
		{
        	float[] actions = new float[7]{0,0,0,0,0,0,0};
			/*Grip*/
			actions[0] = RawToAbsInput(Input.GetAxisRaw("Gripper"));
			/*Position*/
			actions[1] = RawToAbsInput(Input.GetAxisRaw("X"));
			actions[2] = RawToAbsInput(Input.GetAxisRaw("Y"));
			actions[3] = RawToAbsInput(Input.GetAxisRaw("Z"));
			/*Rotation*/
			actions[4] = RawToAbsInput(Input.GetAxisRaw("XRot"));
			actions[5] = RawToAbsInput(Input.GetAxisRaw("YRot"));
			actions[6] = RawToAbsInput(Input.GetAxisRaw("ZRot"));
        	ExecuteMotion(actions);
		}

		ArticulationBody jointAB = gripper.GetABs()[testJoint-1];
		string data = $"{Time.timeAsDouble},{jointAB.jointPosition[0]},{jointAB.jointVelocity[0]},{jointAB.jointAcceleration[0]}";
		Debug.Log(data);
		if(startMonitoring)
		{
			sr.WriteLine(data);
		}
    }

	IEnumerator RepeatMovement(){
		startMonitoring=true;
		float[] actions = new float[7]{0,0,0,0,0,0,0};

		for (int i = 0; i < testRuns; i++)
		{
			ArticulationBody testAB = gripper.GetABs()[testJoint-1];
			//time = 0f;
			actions[testJoint]=1f;
			while (testAB.jointPosition[0]<=targetPosition)
			{
				//Debug.Log(testAB.jointPosition[0]);
				ExecuteMotion(actions);
				yield return new WaitForFixedUpdate();
			}
			actions[testJoint]=0;
			ExecuteMotion(actions);
			yield return new WaitForSecondsRealtime(2f);

			//time = 0f;
			actions[testJoint]=-1f;
			while (testAB.jointPosition[0]>=0)
			{
				ExecuteMotion(actions);
				yield return new WaitForFixedUpdate();
			}
			actions[testJoint]=0;
			ExecuteMotion(actions);
			yield return new WaitForSecondsRealtime(2f);
			//actions[testJoint]=0;
			//ExecuteMotion(actions);
		}
		startMonitoring=false;
		sr.Close();
		#if UNITY_EDITOR
		EditorApplication.isPlaying=false;
		#endif
		yield return true;
	}

    void ExecuteMotion(float[] actions){
        float gripperState = actions[0];
		if (gripperState != 0) {
			gripper.moveGrip(gripperState);
		}
        Vector3 prismaticState = Vector3.zero;
		prismaticState.x = actions[1];
		prismaticState.y = actions[2];
		prismaticState.z = actions[3];

        Vector3 revoluteState = Vector3.zero;
		revoluteState.x = actions[4];
		revoluteState.y = actions[5];
		revoluteState.z = actions[6];


		gripper.moveTCP(prismaticState,revoluteState);
    }
	void OnApplicationQuit()
	{
		if(sr!=null){
			sr.Close();
		}
	}

	float RawToAbsInput(float input) {
		if (input > 0) {
			return 1;
		} else if (input < 0) {
			return -1;
		} else {
			return 0;
		}
	}
}
