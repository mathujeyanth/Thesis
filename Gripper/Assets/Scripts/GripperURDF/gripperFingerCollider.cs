using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class gripperFingerCollider : MonoBehaviour
{
	int counter = 0;
	int maxCount=5;
    [ReadOnly] public bool isGrasped = false;
	[ReadOnly] public bool inCollision = false;
	string targetTag = "brick";
	private void OnCollisionStay(Collision other) {
		if (other.gameObject.CompareTag(targetTag)){
			counter++;
			if(counter>maxCount) isGrasped=true;
		}
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=true;
	}
	private void OnCollisionExit(Collision other) {
		if (other.gameObject.CompareTag(targetTag)){
			counter=0;
			isGrasped=false;
		}
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=false;
	}
}
