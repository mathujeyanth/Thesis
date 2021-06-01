using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TCPControl : GripControl
{

    float linearSpeed = 1f;//0.025f;
	float angularSpeed = 180f;//18f;
    [SerializeField] public ArticulationBody xRevoluteAB,yRevoluteAB,zRevoluteAB,xPrismaticAB,yPrismaticAB,zPrismaticAB;
	//float maxRotation = 360f;
	//float maxPosition = 0.5f;
	Vector3 lastPrismaticSpeed = Vector3.zero;
	Vector3 lastRevoluteSpeed = Vector3.zero;
    protected override void Start()
    {
        base.Start();
    }
    public bool isGripperSleeping()
    {
        return yRevoluteAB.IsSleeping();
    }
	public ArticulationBody[] GetABs()
	{
		ArticulationBody[] ABs = new ArticulationBody[6];
		ABs[0]=xPrismaticAB;
		ABs[1]=yPrismaticAB;
		ABs[2]=zPrismaticAB;
		ABs[3]=xRevoluteAB;
		ABs[4]=yRevoluteAB;
		ABs[5]=zRevoluteAB;

		return ABs;
	}

	public void setTCP(Vector3 prismaticValue,Vector3 revoluteValue) {

		var xRevolute = xRevoluteAB.xDrive;
		var yRevolute = yRevoluteAB.xDrive;
		var zRevolute = zRevoluteAB.xDrive;
		var xPrismatic = xPrismaticAB.xDrive;
		var yPrismatic = yPrismaticAB.xDrive;
		var zPrismatic = zPrismaticAB.xDrive;

		xRevolute.target = Mathf.Clamp(revoluteValue.x,xRevolute.lowerLimit,xRevolute.upperLimit);
		yRevolute.target = Mathf.Clamp(revoluteValue.y,yRevolute.lowerLimit,yRevolute.upperLimit);
		zRevolute.target = Mathf.Clamp(revoluteValue.z,zRevolute.lowerLimit,zRevolute.upperLimit);
		xPrismatic.target = Mathf.Clamp(prismaticValue.x,xPrismatic.lowerLimit,xPrismatic.upperLimit);
		yPrismatic.target = Mathf.Clamp(prismaticValue.y,yPrismatic.lowerLimit,yPrismatic.upperLimit);
		zPrismatic.target = Mathf.Clamp(prismaticValue.z,zPrismatic.lowerLimit,zPrismatic.upperLimit);

		//Debug.Log(yPrismatic.target + " " + yPrismaticAB.jointPosition[0]);

		if(xRevoluteAB.xDrive.target != xRevolute.target) xRevoluteAB.xDrive = xRevolute;
		if(yRevoluteAB.xDrive.target != yRevolute.target) yRevoluteAB.xDrive = yRevolute;
		if(zRevoluteAB.xDrive.target != zRevolute.target) zRevoluteAB.xDrive = zRevolute;
		if(xPrismaticAB.xDrive.target != xPrismatic.target) xPrismaticAB.xDrive = xPrismatic;
		if(yPrismaticAB.xDrive.target != yPrismatic.target) yPrismaticAB.xDrive = yPrismatic;
		if(zPrismaticAB.xDrive.target != zPrismatic.target) zPrismaticAB.xDrive = zPrismatic;
		//yRevoluteAB.xDrive = yRevolute;
		//xPrismaticAB.xDrive = xPrismatic;
		//yPrismaticAB.xDrive = yPrismatic;
		//zPrismaticAB.xDrive = zPrismatic;
	}

	public void moveTCP(Vector3 prismaticDirection,Vector3 revoluteDirection) {
        ////Vector3 drivePosition = Vector3.zero;
        ////drivePosition.x = xPrismaticAB.xDrive.target;//xPrismaticAB.jointPosition[0];
        ////drivePosition.y = yPrismaticAB.xDrive.target;//yPrismaticAB.jointPosition[0];
        ////drivePosition.z = zPrismaticAB.xDrive.target;//zPrismaticAB.jointPosition[0];
        ////drivePosition += prismaticDirection* Time.fixedDeltaTime * linearSpeed;

        ////Vector3 driveRotation = Vector3.zero;
        ////driveRotation.y = yRevoluteAB.xDrive.target;//yRevoluteAB.jointPosition[0]* Mathf.Rad2Deg;
        ////driveRotation += revoluteDirection* Time.fixedDeltaTime * angularSpeed;

        ///*Position*/
        //Vector3 drivePosition = Vector3.zero;
        //drivePosition.x = xPrismaticAB.jointPosition[0];//xPrismaticAB.xDrive.target;
        //drivePosition.y = yPrismaticAB.jointPosition[0];//yPrismaticAB.xDrive.target;
        //drivePosition.z = zPrismaticAB.jointPosition[0];//zPrismaticAB.xDrive.target;

		//Vector2 increments = Vector2.zero;
		//increments[0] = prismaticDirection.x* Time.fixedDeltaTime * linearSpeed;
		//increments[1] = prismaticDirection.z* Time.fixedDeltaTime * linearSpeed;

		//Vector2 rotatedXZ = Vector2.zero;
		//float angle = - yRevoluteAB.jointPosition[0];//- yRevoluteAB.xDrive.target*Mathf.Deg2Rad;
		//rotatedXZ[0] = increments[0]*Mathf.Cos(angle) - increments[1]*Mathf.Sin(angle);
		//rotatedXZ[1] = increments[0]*Mathf.Sin(angle) + increments[1]*Mathf.Cos(angle);

		//drivePosition.x += rotatedXZ.x;
        //drivePosition.y += prismaticDirection.y* Time.fixedDeltaTime * linearSpeed;
		//drivePosition.z += rotatedXZ.y;
		////Debug.Log($"{prismaticDirection} --> {increments.ToString("F5")} --> {rotatedXZ.ToString("F5")} {angle}");

        ///*Rotation*/
        //Vector3 driveRotation = Vector3.zero;
        //driveRotation.y = yRevoluteAB.jointPosition[0] * Mathf.Rad2Deg;//yRevoluteAB.xDrive.target;
        //driveRotation += revoluteDirection* Time.fixedDeltaTime * angularSpeed;

        ///*Ensures y prismatic(gravity) is only set if moved by player*/
		////Debug.Log($"{yPrismatic.target} {yPrismaticAB.jointPosition[0]}");
		////setTCP(drivePosition,driveRotation); // <-- this does not work
        //var yRevolute = yRevoluteAB.xDrive;
		//var xPrismatic = xPrismaticAB.xDrive;
		//var yPrismatic = yPrismaticAB.xDrive;
		//var zPrismatic = zPrismaticAB.xDrive;

		//yRevolute.target = Mathf.Clamp(driveRotation.y,yRevolute.lowerLimit,yRevolute.upperLimit);
		//xPrismatic.target = Mathf.Clamp(drivePosition.x,xPrismatic.lowerLimit,xPrismatic.upperLimit);
		//yPrismatic.target = Mathf.Clamp(drivePosition.y,yPrismatic.lowerLimit,yPrismatic.upperLimit);
		//zPrismatic.target = Mathf.Clamp(drivePosition.z,zPrismatic.lowerLimit,zPrismatic.upperLimit);

		//if(revoluteDirection.y != 0f) yRevoluteAB.xDrive = yRevolute;
		//if(rotatedXZ[0] != 0f) xPrismaticAB.xDrive = xPrismatic;
		//if(prismaticDirection.y != 0f) yPrismaticAB.xDrive = yPrismatic;
		//if(rotatedXZ[1] != 0f) zPrismaticAB.xDrive = zPrismatic;

        Vector3 drivePosition = Vector3.zero;
        drivePosition.x = prismaticDirection.x == 0f ? xPrismaticAB.xDrive.target : xPrismaticAB.jointPosition[0];
        drivePosition.y = prismaticDirection.y == 0f ? yPrismaticAB.xDrive.target : yPrismaticAB.jointPosition[0];
        drivePosition.z = prismaticDirection.z == 0f ? zPrismaticAB.xDrive.target : zPrismaticAB.jointPosition[0];
        drivePosition += prismaticDirection* Time.fixedDeltaTime * linearSpeed;

		//Vector3 prismaticSpeed = lastPrismaticSpeed+prismaticDirection*Time.fixedDeltaTime*0.12f;//RandomGaussian(minValue:0.08f,maxValue:0.12f);//0.1f; 
		//prismaticSpeed.x = prismaticDirection.x==0f ? Mathf.Pow(prismaticSpeed.x,2)/(2*0.12f) : Mathf.Clamp(prismaticSpeed.x,-linearSpeed,linearSpeed);
		//prismaticSpeed.y = prismaticDirection.y==0f ? Mathf.Pow(prismaticSpeed.y,2)/(2*0.12f) : Mathf.Clamp(prismaticSpeed.y,-linearSpeed,linearSpeed);
		//prismaticSpeed.z = prismaticDirection.z==0f ? Mathf.Pow(prismaticSpeed.z,2)/(2*0.12f) : Mathf.Clamp(prismaticSpeed.z,-linearSpeed,linearSpeed);
        //drivePosition += prismaticSpeed;
		//lastPrismaticSpeed = prismaticSpeed;


        Vector3 driveRotation = Vector3.zero;
        driveRotation.x = revoluteDirection.x == 0f ? xRevoluteAB.xDrive.target : xRevoluteAB.jointPosition[0]* Mathf.Rad2Deg;
        driveRotation.y = revoluteDirection.y == 0f ? yRevoluteAB.xDrive.target : yRevoluteAB.jointPosition[0]* Mathf.Rad2Deg;
        driveRotation.z = revoluteDirection.z == 0f ? zRevoluteAB.xDrive.target : zRevoluteAB.jointPosition[0]* Mathf.Rad2Deg;
        driveRotation += revoluteDirection* Time.fixedDeltaTime * angularSpeed;

		//Vector3 revoluteSpeed = lastRevoluteSpeed+revoluteDirection*Time.fixedDeltaTime*1.8f;//RandomGaussian(minValue:0.08f,maxValue:0.12f);//0.1f; 
		//revoluteSpeed.x = revoluteDirection.x==0f ? 0f : Mathf.Clamp(revoluteSpeed.x,-angularSpeed,angularSpeed);
		//revoluteSpeed.y = revoluteDirection.y==0f ? 0f : Mathf.Clamp(revoluteSpeed.y,-angularSpeed,angularSpeed);
		//revoluteSpeed.z = revoluteDirection.z==0f ? 0f : Mathf.Clamp(revoluteSpeed.z,-angularSpeed,angularSpeed);
        //driveRotation += revoluteSpeed;
		//lastRevoluteSpeed = revoluteSpeed;

		setTCP(drivePosition,driveRotation);
	}
	public static float RandomGaussian(float minValue = 0.0f, float maxValue = 1.0f)
     {
         float u, v, S;
     
         do
         {
             u = 2.0f * UnityEngine.Random.value - 1.0f;
             v = 2.0f * UnityEngine.Random.value - 1.0f;
             S = u * u + v * v;
         }
         while (S >= 1.0f);
     
         // Standard Normal Distribution
         float std = u * Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);
     
         // Normal Distribution centered between the min and max value
         // and clamped following the "three-sigma rule"
         float mean = (minValue + maxValue) / 2.0f;
         float sigma = (maxValue - mean) / 3.0f;
         return Mathf.Clamp(std * sigma + mean, minValue, maxValue);
     }
}
