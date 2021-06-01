using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class fingerCollider : MonoBehaviour {
	BoxCollider _fingerCollider;
	[ReadOnly] public bool touchedBrick;
	[ReadOnly] public bool inCollision;
	string targetTag = "brick";
	private void Awake() {
		_fingerCollider = this.GetComponent<BoxCollider>();
	}

	//private void OnCollisionEnter(Collision other) {
	//	if (other.gameObject.CompareTag(targetTag)) touchedBrick=true;
	//	if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=true;
	//}
	private void OnCollisionStay(Collision other) {
		if (other.gameObject.CompareTag(targetTag)) touchedBrick=true;
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=true;
	}
	private void OnCollisionExit(Collision other) {
		if (other.gameObject.CompareTag(targetTag)) touchedBrick=false;
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=false;
	}
}
