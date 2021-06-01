using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class BrickFloorCreator : MonoBehaviour {
	public GameObject nob;
	public bool CreateNobs = false;
	public bool DestroyNobs = false;
	Vector3 floorDims;
	// Start is called before the first frame update
	void Start() {
		floorDims = gameObject.GetComponent<Collider>().bounds.size;
	}

	void CreateNobFloor() {
		float scaleFactor=1f;
		floorDims = gameObject.GetComponent<Collider>().bounds.size;
		int amountInX = (int)(floorDims.x / (0.08f*scaleFactor));
		int amountInZ = (int)(floorDims.z / (0.08f*scaleFactor));
		for (int i = 0; i < amountInX; i++) {
			for (int j = 0; j < amountInZ; j++) {
				GameObject tmp = Instantiate(nob, transform.position, Quaternion.identity);
				tmp.transform.localScale *= scaleFactor;
				//Vector3 newPos = transform.position-new Vector3(floorDims.x/0.8f,0f,floorDims.z/0.8f) + new Vector3(0.04f+0.08f*(float)i,0.005f,0.04f+0.08f*(float)j);
				tmp.transform.parent = gameObject.transform;
				Vector3 newPos = new Vector3(scaleFactor*0.04f + scaleFactor*0.08f * (float)(i - amountInX / 2), 0.008f+0.5f*floorDims.y, scaleFactor*0.04f + scaleFactor*0.08f * (float)(j - amountInZ / 2));
				//Debug.Log(newPos.ToString("F2"));
				tmp.transform.position += newPos;
				tmp.name = nob.name + "_" + i.ToString("00") + "-" + j.ToString("00");
				tmp.GetComponent<MeshRenderer>().material = GetComponent<MeshRenderer>().sharedMaterial;
			}
		}
	}
	void DestroyNobFloor() {
		foreach (var child in GetComponentsInChildren<Transform>()) {
			if (child.CompareTag("brickNob")) {
				DestroyImmediate(child.gameObject);
			}
		}
	}
	void Update() {
		if (CreateNobs) {
			CreateNobs = false;
			DestroyNobFloor();
			CreateNobFloor();
		}
		if (DestroyNobs) {
			DestroyNobs = false;
			DestroyNobFloor();
		}
	}
}
