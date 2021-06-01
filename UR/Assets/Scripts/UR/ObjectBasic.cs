using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectBasic : MonoBehaviour {
    //public struct Object
    //{
    //    public Vector3 StartPosition;
    //    public Quaternion StartRotation;
    //    public Vector3 TargetPosition;
    //    public Quaternion TargetRotation;
    //    public float DistancePositionStartToTarget;
    //    public float DistanceRotationStartToTarget;     
    //    public bool IsConnected;
    //    public bool RemovedFromTarget;
	//    bool completed;
    //    public Object(Vector3 _StartPosition,Quaternion _StartRotation,Vector3 _TargetPosition,Quaternion _TargetRotation,float _DistancePositionStartToTarget, float _DistanceRotationStartToTarget){
    //        StartPosition = _StartPosition;
    //        StartRotation = _StartRotation;
    //        TargetPosition = _TargetPosition;
    //        TargetRotation = _TargetRotation;
    //        DistancePositionStartToTarget = _DistancePositionStartToTarget;
    //        DistanceRotationStartToTarget = _DistanceRotationStartToTarget;
    //        IsConnected = false;
    //        RemovedFromTarget = false;
    //        completed = false;
    //    }
    //}
	public Vector3 StartPosition;
	public Quaternion StartRotation;
	public Vector3 TargetPosition;
	public Quaternion TargetRotation;
	public float DistancePositionStartToTarget;
	public float DistanceRotationStartToTarget;

	public bool IsCompleted = false;
	public bool RemovedFromTarget = false;
	public Rigidbody rb;
	[SerializeField] protected bool IsConnected=false;
	public virtual float height {get;}=0.05f;
	string tagName = "Untagged";

	//public bool IsConnected=false;
	void Awake() {
		tagName = gameObject.tag;
		rb = GetComponent<Rigidbody>();	
	}
	void Start() {
		StartPosition = transform.position;
		StartRotation = transform.rotation;
	}
    public void setColor(Color color){
        if(TryGetComponent<MeshRenderer>(out MeshRenderer meshRenderer)){
            meshRenderer.material.color=color;
        }
        foreach (var childMesh in GetComponentsInChildren<MeshRenderer>())
        {
            childMesh.material.color = color;
        }
    }
	public void resetTag(){
		gameObject.tag = tagName;
	}
	public void removeTag(){
		gameObject.tag = "Untagged";
	}

	public Vector3 positionDistance(float yMultiplier=1f){
		return Vector3.Scale((transform.position-TargetPosition), new Vector3(1f, yMultiplier, 1f));
	}
	public float rotationDistance(){
		return Quaternion.Angle(transform.rotation, TargetRotation) / 180f;
	}
	//public float positionDistanceStart(){
	//	return Vector3.Scale((transform.position-StartPosition), new Vector3(1f, 0f, 1f)).magnitude;
	//}
	//public float rotationDistanceStart(){
	//	return Quaternion.Angle(transform.rotation, StartRotation) / 180f;
	//}
	public float calculateDistancePosition(float yMultiplier=0f){
		StartPosition = transform.position;
		DistancePositionStartToTarget = Vector3.Scale((StartPosition-TargetPosition), new Vector3(1f, yMultiplier, 1f)).magnitude;
		return DistancePositionStartToTarget;		
	}
	//public float calculateDistanceRotation(){
	//	StartRotation = transform.rotation;
	//	DistanceRotationStartToTarget = Quaternion.Angle(TargetRotation, StartRotation) / 180f;
	//	return DistanceRotationStartToTarget;
	//}

	public bool isPlaced(){
		if(positionDistance(yMultiplier:0f).AbsCompare(0.02f,lessThan:true) && 
			rotationDistance() <= 0.01f && IsConnected){
			return true;
			//IsPlaced = true;
		}
		return false;
	}
	public bool isRemovedFromTarget(){
        if (/*!IsConnected ||*/ positionDistance(yMultiplier:0f).AbsCompare(0.02f,lessThan:false)/*5mm*/ || rotationDistance() > 0.05f) {
			return true;
		}	
		return false;
	}

	private void FixedUpdate() {
		//isPlaced();
		RemovedFromTarget = IsCompleted ? isRemovedFromTarget() : false;
	}
}
