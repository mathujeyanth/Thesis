using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperFingerCollider : MonoBehaviour
{
    public bool isGrasped = false;
	public bool inCollision = false;
	string targetTag = "brick";
	private void OnCollisionStay(Collision other) {
		if (other.gameObject.CompareTag(targetTag)) isGrasped=true;
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=true;
	}
	private void OnCollisionExit(Collision other) {
		if (other.gameObject.CompareTag(targetTag)) isGrasped=false;
		if (!(other.gameObject.CompareTag(targetTag) || other.gameObject.CompareTag("gripper"))) inCollision=false;
	}
}
