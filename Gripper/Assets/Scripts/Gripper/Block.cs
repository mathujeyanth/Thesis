using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Block : ObjectBasic {
	void OnCollisionStay(Collision other) {
		if(other.gameObject.CompareTag("brick") || other.gameObject.CompareTag("brickFloor")){
			if(IsConnected) return;
			Vector3 hit = other.GetContact(0).normal;
			float angle = Vector3.Angle(hit,Vector3.forward);
			if(90f-angle<0.01f){
				IsConnected=true;
			}
		}
	}

	//private void OnCollisionEnter(Collision other) {
	//	if(other.gameObject.CompareTag("brick")){
	//		Vector3 hit = other.GetContact(0).normal;
	//		float angle = Vector3.Angle(hit,Vector3.forward);
	//		if(90f-angle<0.05f){
	//			//Debug.Log("Placed");
	//			placedOnBrick=true;
	//		}
	//	}
	//}
	private void OnCollisionExit(Collision other) {
		if(other.gameObject.CompareTag("brick") || other.gameObject.CompareTag("brickFloor")){
			IsConnected=false;
		}
	}
	
}
