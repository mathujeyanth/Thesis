using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

enum BrickType {B222, B242, B212}; //B112, B212, B222, B322, B422, B112, B212, B111,B211, B221, B321, B421 //B_LENGTH_WIDTH_HEIGHT
enum BrickColor {random,black,white,green,blue,red,yellow};
public class BrickManager : MonoBehaviour {
	public List<GameObject> brickTypes;
	[ReadOnly] public List<Color> brickColors = new List<Color>();
	[ReadOnly] public List<ObjectBasic> bricksInstantiated = new List<ObjectBasic>();
	public GameObject pickArea;
	public GameObject placeArea;
	[ReadOnly,SerializeField] bool placedBrickRemoved = false;
	Bounds pickBounds;
	Bounds placeBounds;
	int currentBrick = -1;
	[ReadOnly,SerializeField] bool allBricksPlaced = false;
	//[SerializeField] bool destroy = false;

	// Start is called before the first frame update
	private void Awake() {
		brickColors.Add(new Color(0f,0f,0f));  //Black
		brickColors.Add(new Color(244f/255f,244f/255f,244f/255f));  //White
		brickColors.Add(new Color(67f/255f,177f/255f,77f/255f));  //Green
		brickColors.Add(new Color(42f/255f,113f/255f,189f/255f));  //Blue
		brickColors.Add(new Color(212f/255f,35f/255f,0f/255f));  //Red
		brickColors.Add(new Color(250f/255f,206f/255f,0f/255f)); //Yellow
	}
	public void AddBrick(int brickType, int color, bool gravityOn) {
		GameObject newBrick = Instantiate(brickTypes[brickType], Vector3.zero, Quaternion.identity);
		newBrick.transform.parent = transform;
		newBrick.GetComponent<Rigidbody>().useGravity=gravityOn;

		if(color == (int)BrickColor.random) color = Random.Range(1,(int)System.Enum.GetNames(typeof(BrickColor)).Length-1); // Random color from existing colours
		//newBrick.GetComponent<Renderer>().material.color = brickColors[color];
		//if(color == (int)BrickColor.random) newBrick.GetComponent<Renderer>().material.color = Random.ColorHSV(0f,1f,1f,1f,0.5f,1f); //Generates random colours

		//newBrick.AddComponent<ObjectBasic>();
		newBrick.GetComponent<ObjectBasic>().setColor(brickColors[color]);
		bricksInstantiated.Add(newBrick.GetComponent<ObjectBasic>());
		if( bricksInstantiated.Count == 1) currentBrick = 0;
	}
	public void DestroyAllBricks() {
		foreach (ObjectBasic brick in bricksInstantiated){
			//DestroyImmediate(brick.gameObject);
			/*Safer than destroy immediate*/
			brick.gameObject.transform.parent = null;
			brick.gameObject.name = "$destroyed";
			Destroy(brick.gameObject);
			brick.gameObject.SetActive(false);
		}
		bricksInstantiated = new List<ObjectBasic>();
		currentBrick = -1;
	}

	public ObjectBasic GetCurrentBrick() {
		if (currentBrick == -1){
			Debug.LogError("No bricks in list.");
			return null;
		}
		if (bricksInstantiated[currentBrick].IsCompleted && currentBrick < bricksInstantiated.Count) {
			currentBrick++;
		}

		return bricksInstantiated[currentBrick];
	}
	public bool placedAllBricks(){
		foreach (ObjectBasic brick in bricksInstantiated) {
			if(!brick.IsCompleted) return false;
		}
		return true;
	}
	public bool placedAndRemoved(){
		foreach (ObjectBasic brick in bricksInstantiated) {
			if (brick.RemovedFromTarget) return true;
		}
		return false;
	}


	//void FixedUpdate() {
	//	foreach (Brick brick in bricksInstantiated) {
	//		allBricksPlaced=false;
	//		if (brick.RemovedFromTarget) {
	//			placedBrickRemoved = true;
	//			break;
	//		}
	//		if(brick.IsPlaced) allBricksPlaced=true;
	//	}
	//	if (destroy) {
	//		DestroyAllBricks();
	//	}
	//}
}
