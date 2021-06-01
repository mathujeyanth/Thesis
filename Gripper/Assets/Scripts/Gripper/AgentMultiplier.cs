using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class AgentMultiplier : MonoBehaviour {
	public GameObject environment;
	public int Amount = 0;
	[ReadOnly] public int RealAmount = 0;
	public bool CreateEnvironments = false;
	public bool DeleteEnvironments = false;
	int cloneCounter=0;

	void DestroyEnvs(){
		foreach (GameObject env in GameObject.FindGameObjectsWithTag("clone")) {
			DestroyImmediate(env);
		}
		cloneCounter=0;
	}
	void CreateEnvs(){
		Vector3 EnvPosition = environment.transform.position;
		int x = 0, y = 0;
		for (int i = 0; i < Amount; i++) {
			y=0;
			x=i;
			for (int j = 0; j < Amount; j++) {
				y=j;
				if(x==0 && y==0) continue;
				//GameObject newEnvironment = Instantiate(environment, new Vector3(-28f*x,0,28f*y), Quaternion.identity);
				GameObject newEnvironment = Instantiate(environment, EnvPosition+ (new Vector3(-3f*x,0,3f*y)), Quaternion.identity);
				newEnvironment.tag="clone";
				cloneCounter++;
				newEnvironment.name=environment.name+"_"+cloneCounter.ToString("000");
				//Debug.Log(x+""+y);

				var agentScript1 = newEnvironment.GetComponentInChildren<GripperAgent>();
				if(agentScript1!=null){
					agentScript1.showDebugText = false;
					//gameObject.layer
				}
				var agentScript2 = newEnvironment.GetComponentInChildren<RobotiqAgent>();
				if(agentScript2!=null){
					agentScript2.agentLayer+=cloneCounter;
					agentScript2.showDebugText = false;
				}
			}
		}
	}

	// Update is called once per frame
	void Update() {
        if (Application.isPlaying != true) RealAmount=Amount*Amount;
		if (Application.isPlaying != true && CreateEnvironments) {
			CreateEnvironments = false;
			DestroyEnvs();
			CreateEnvs();
		}
		if (Application.isPlaying != true && DeleteEnvironments) {
			DeleteEnvironments = false;
			DestroyEnvs();
		}
	}
}
