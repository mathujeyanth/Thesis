using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR
	
public class ScreenshotGrabber
{
	[MenuItem("Screenshot/Grab")]
	public static void Grab()
	{
		ScreenCapture.CaptureScreenshot("Screenshot.png", 1);
	}
}
#endif

public static class Extensions
{
    //[RuntimeInitializeOnLoadMethod]
    //private static void turnOffLogger(){
    //    Unity.Simulation.TimeLogger.logWallTime = false;
    //    Unity.Simulation.TimeLogger.logFrameTiming = false;
    //    Unity.Simulation.TimeLogger.logSimulationTime = false;
    //    Unity.Simulation.TimeLogger.logUnscaledSimulationTime = false;
    //    Unity.Simulation.TimeLogger.logWallTime = false;
    //}
	public static Quaternion ShortestRotation(this Quaternion a, Quaternion b) {
		if (UnityEngine.Quaternion.Dot(a, b) < 0) {
			return a * Quaternion.Inverse(new Quaternion(-1f * b.x, -1f * b.y, -1f * b.z, -1f * b.w));
		} else return a * Quaternion.Inverse(b);
	}
	public static Vector3 EulerLimit(this Vector3 a) {
        Vector3 tmp = a;
        for (int i = 0; i < 3; i++){
            if(a[i]>180f) tmp[i]=-360f+a[i];
            else if(a[i]<-180f) tmp[i]=360f+a[i];
        }
        return tmp;
	}
	public static bool AbsCompare(this Vector3 a,float value,bool lessThan=true) {
        bool tmp = true;
        for (int i = 0; i < 3; i++){
            if(lessThan && Mathf.Abs(a[i])>value) tmp = false;
            else if(!lessThan && Mathf.Abs(a[i])<value) tmp = false;
        }
        return tmp;
	}

}
