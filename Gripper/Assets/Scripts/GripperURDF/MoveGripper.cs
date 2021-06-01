using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum BigHandState { Fixed = 0, MovingUp = 1, MovingDown = -1 };
public class MoveGripper : MonoBehaviour
{

    public BigHandState moveState = BigHandState.Fixed;
    float speed = 1f;
    float accelerationMax = 1f;
    float speedMax = 1f;
    private BigHandState lastMoveState = BigHandState.Fixed;
    ArticulationBody handAB;
    public ArticulationBody LeftFingerAB;
    public ArticulationBody RightFingerAB;
    float velocityT;
    bool resetState = false;
    float minPrecisionValue=0.00001f;
    void openGripper(){
        var leftFingerDrive = LeftFingerAB.xDrive;
        var rightFingerDrive = RightFingerAB.xDrive;
        leftFingerDrive.target = 0f;
        rightFingerDrive.target = 0f;
        LeftFingerAB.xDrive = leftFingerDrive;
        RightFingerAB.xDrive = rightFingerDrive;            
    }
    void closeGripper(){
        var leftFingerDrive = LeftFingerAB.xDrive;
        var rightFingerDrive = RightFingerAB.xDrive;
        leftFingerDrive.target = 0.25f;
        rightFingerDrive.target = 0.25f;
        LeftFingerAB.xDrive = leftFingerDrive;
        RightFingerAB.xDrive = rightFingerDrive;
    }
    void Start() {
        handAB = GetComponent<ArticulationBody>();
    }
    void moveGripper(float direction){
        float xDrivePostion = handAB.jointPosition[0];
        //get jointPosition along y axis
        //Debug.Log(xDrivePostion);

        //increment this y position
        float targetPosition = xDrivePostion + -direction * Time.fixedDeltaTime * speed;

        //set joint Drive to new position
        var drive = handAB.xDrive;
        drive.target = targetPosition;
        handAB.xDrive = drive;
    }

    private void FixedUpdate()
    {
        if(!resetState){
            if(Input.GetKeyDown("0")){ //Reset state
                Debug.Log("0 pressed");

                var drive = handAB.xDrive;
                drive.target = 0f;
                handAB.xDrive = drive;

                openGripper();
                resetState = true;
            }

            if(Input.GetKeyDown("k")){
                openGripper();
            }
            else if(Input.GetKeyDown("l")){
                closeGripper();
            }

            float input = Input.GetAxisRaw("ZRot");
            moveState = MoveStateForInput(input);
            if (moveState != BigHandState.Fixed){
                moveGripper((float)moveState);
            }
        }
        else
        {   
            if (handAB.jointPosition[0]<minPrecisionValue 
                && LeftFingerAB.jointPosition[0]<minPrecisionValue 
                && RightFingerAB.jointPosition[0]<minPrecisionValue)
            {
                resetState = false;
                Debug.Log("finished");
            }
        }


        

        ///*V0*/
        //if (moveState != BigHandState.Fixed)
        //{

        //    ArticulationBody articulation = GetComponent<ArticulationBody>();
        //    float xDrivePostion = articulation.jointPosition[0];
        //    //get jointPosition along y axis
        //    Debug.Log(xDrivePostion);

        //    //increment this y position
        //    float targetPosition = xDrivePostion + -(float)moveState * Time.fixedDeltaTime * speed;

        //    //set joint Drive to new position
        //    var drive = articulation.xDrive;
        //    drive.target = targetPosition;
        //    articulation.xDrive = drive;
        //}


        ///*V1*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;

        //if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //    return;
        //}

        //articulation.jointType = ArticulationJointType.PrismaticJoint;
        //float targetVelocity = -(float)moveState * Time.fixedDeltaTime * 10f;
        //drive.targetVelocity = targetVelocity;
        //articulation.xDrive = drive;

        //gameObject.SetActive(false);
        //gameObject.SetActive(true);

        ///*V2*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;

        //if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //}
        //else
        //{
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float newVelocity = drive.targetVelocity - (float)moveState * Time.fixedDeltaTime * accelerationMax;
        //    float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;

        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //}


        ///*V3*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;

        ////if (moveState == BigHandState.Fixed && !Mathf.Approximately(drive.targetVelocity,0f))
        ////{
        ////    articulation.jointType = ArticulationJointType.PrismaticJoint;
        ////    float newVelocity = Mathf.Pow(drive.targetVelocity,2)/(2*accelerationMax);
        ////    float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);
        ////    drive.targetVelocity = targetVelocity;
        ////    articulation.xDrive = drive;
        ////    Debug.Log(articulation.jointVelocity[0]);

        ////    gameObject.SetActive(false);
        ////    gameObject.SetActive(true);
        ////}
        //if (moveState == BigHandState.Fixed && articulation.jointVelocity.dofCount > 0 && !Mathf.Approximately(articulation.jointVelocity[0],0f))
        //{
        //    float currentVelocity = articulation.jointVelocity[0];
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float newVelocity = currentVelocity - Mathf.Sign(currentVelocity) * Time.fixedDeltaTime * accelerationMax;
        //    float targetVelocity = Mathf.Sign(currentVelocity)>0f ? Mathf.Clamp(newVelocity,0f,speedMax) : Mathf.Clamp(newVelocity,-speedMax,0f);;
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;
        //    //Debug.Log(articulation.jointVelocity[0]);

        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //}
        //else if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //    //Debug.Log(articulation.jointVelocity.dofCount);
        //}
        //else if (moveState != BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float newVelocity = drive.targetVelocity - (float)moveState * Time.fixedDeltaTime * accelerationMax;
        //    float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;
        //    //Debug.Log(articulation.jointVelocity[0]);

        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //}

        ///*V4*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;


        //if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //}
        //else if (moveState != BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float actualVelocity = -(float)articulation.jointVelocity[0];
        //    float driveVelocity = drive.targetVelocity;
        //    float currentVelocity = !(Mathf.Abs(actualVelocity)<0.001f) ? driveVelocity : 0f;
        //    float newVelocity = currentVelocity + (float)moveState * Time.fixedDeltaTime * accelerationMax;
        //    Debug.Log(currentVelocity+" "+drive.targetVelocity+" "+actualVelocity);
        //    float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;

        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //}
        ///*V5*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;

        //if (moveState != lastMoveState)
        //{
        //    lastMoveState = moveState;
        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //    //gameObject.transform.parent.gameObject.SetActive(false);
        //    //gameObject.transform.parent.gameObject.SetActive(true);
        //}

        //if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //}
        //else
        //{
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float targetVelocity = -(float)moveState * Time.fixedDeltaTime * 10f;
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;
        //}

        ///*V6*/
        //ArticulationBody articulation = GetComponent<ArticulationBody>();
        //var drive = articulation.xDrive;


        //if (moveState != lastMoveState)
        //{
        //    lastMoveState = moveState;
        //    //gameObject.transform.parent.gameObject.SetActive(false);
        //    //gameObject.transform.parent.gameObject.SetActive(true);
        //    //gameObject.SetActive(false);
        //    //gameObject.SetActive(true);
        //}
        //if (moveState == BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.FixedJoint;
        //    drive.targetVelocity = 0;
        //    articulation.xDrive = drive;
        //}
        //else if (moveState != BigHandState.Fixed)
        //{
        //    articulation.jointType = ArticulationJointType.PrismaticJoint;
        //    float actualVelocity = -(float)articulation.jointVelocity[0];
        //    float driveVelocity = drive.targetVelocity;
        //    float currentVelocity = !(Mathf.Abs(actualVelocity)<0.001f) ? driveVelocity : 0f;
        //    float newVelocity = currentVelocity + (float)moveState * Time.fixedDeltaTime * accelerationMax;
        //    Debug.Log(currentVelocity+" "+drive.targetVelocity+" "+actualVelocity);
        //    float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);
        //    drive.targetVelocity = targetVelocity;
        //    articulation.xDrive = drive;

        //    gameObject.SetActive(false);
        //    gameObject.SetActive(true);
        //}
        
        ///*V8*/
        //if (moveState != BigHandState.Fixed)
        //{
        //    ArticulationBody articulation = GetComponent<ArticulationBody>();

        //    //get jointPosition along y axis
        //    //float newVelocity = articulation.jointVelocity[0] - (float)moveState * Time.fixedDeltaTime * accelerationMax;
        //    //float targetVelocity = Mathf.Clamp(newVelocity,-speedMax,speedMax);

        //    float xDrivePostion = articulation.jointPosition[0];
        //    float xDriveVelocity = articulation.jointVelocity[0];
        //    Debug.Log($"{xDrivePostion} {xDriveVelocity}");

        //    //increment this y position
        //    float targetPosition = xDrivePostion + velocityT + (float)moveState *  Time.fixedDeltaTime * accelerationMax;
        //    velocityT = Mathf.Clamp(articulation.jointVelocity[0] + (float)moveState * Time.fixedDeltaTime * accelerationMax,-speedMax,speedMax);;

        //    //set joint Drive to new position
        //    var drive = articulation.xDrive;
        //    drive.target = targetPosition;
        //    articulation.xDrive = drive;
        //}
    }

    BigHandState MoveStateForInput(float input)
    {
        if (input > 0)
        {
            return BigHandState.MovingUp;
        }
        else if (input < 0)
        {
            return BigHandState.MovingDown;
        }
        else
        {
            return BigHandState.Fixed;
        }
    }
}
