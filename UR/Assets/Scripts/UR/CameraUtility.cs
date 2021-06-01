using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.UrMoveit;
using RosMessageTypes.Control;
using RosMessageTypes.Sensor;

public class CameraUtility : MonoBehaviour
{
	///*ROS*/
    //private ROSConnection ros;
	//[SerializeField, ReadOnly] string PUB_VISUAL_OBS = "ur_visual_obs";
	//[SerializeField, ReadOnly] string PUB_VECTOR_OBS = "ur_vector_obs";

	/*Camera*/
	public Camera camera;
	[SerializeField] RenderTexture renderTexture;
	private const int isBigEndian = 0;
	private const int step = 4;
	//public PoseEstimationScenario scenario;

    // Start is called before the first frame update
    void Start()
    {
		// Get ROS connection static instance
		//ros = ROSConnection.instance;
		// Render texture 
		renderTexture = new RenderTexture(camera.pixelWidth, camera.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
		renderTexture.Create();
		//StartCoroutine(PublishCamera(0.008f));//125Hz stated in doc
    }
	
    /// <summary>
	///     Capture the main camera's render texture and convert to bytes.
	/// </summary>
	/// <returns>imageBytes</returns>
	private byte[] CaptureScreenshot()
	{
		camera.targetTexture = renderTexture;
		RenderTexture currentRT = RenderTexture.active;
		RenderTexture.active = renderTexture;
		camera.Render();
		Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
		mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
		mainCameraTexture.Apply();
		RenderTexture.active = currentRT;
		// Get the raw byte info from the screenshot
		byte[] imageBytes = mainCameraTexture.GetRawTextureData();
		camera.targetTexture = null;
		return imageBytes;
	}

	/// <summary>
	///     Button callback for the Pose Estimation
	/// </summary>
	public byte[] PoseEstimation()
	{
		// Capture the screenshot and pass it to the pose estimation service
		//byte[] rawImageData = CaptureScreenshot();
		//return createImage(rawImageData);
		int width = 84;//camera.pixelWidth;
		int height = 84;//camera.pixelHeight;
		//Debug.Log($"{width} {height}");
		var texture2D = new Texture2D(width, height, TextureFormat.RGB24,false);
		var oldRec = camera.rect;
		camera.rect = new Rect(0f, 0f, 1f, 1f);
		var depth = 24;
		var format = RenderTextureFormat.Default;
		var readWrite = RenderTextureReadWrite.Default;

		var tempRt = RenderTexture.GetTemporary(width, height, depth, format, readWrite);

		var prevActiveRt = RenderTexture.active;
		var prevCameraRt = camera.targetTexture;

		// render to offscreen texture (readonly from CPU side)
		RenderTexture.active = tempRt;
		camera.targetTexture = tempRt;

		camera.Render();

		texture2D.ReadPixels(new Rect(0, 0, texture2D.width, texture2D.height), 0, 0);

		camera.targetTexture = prevCameraRt;
		camera.rect = oldRec;
		RenderTexture.active = prevActiveRt;
		RenderTexture.ReleaseTemporary(tempRt);

		var imageData = texture2D.EncodeToPNG();

		Object.Destroy(texture2D);
		return imageData;
		//uint imageHeight = (uint)renderTexture.height;
		//uint imageWidth = (uint)renderTexture.width;
		//MCompressedImage rosImage = new MCompressedImage(new RosMessageTypes.Std.MHeader(), "png", imageData);
		////MImage rosImage = new MImage(new RosMessageTypes.Std.MHeader(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
		//return rosImage;
	}

	///// <summary>
	/////     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
	/////     a sensor_msgs/image.
	/////
	/////     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
	/////     PoseEstimationServiceResponse.
	///// </summary>
	///// <param name="imageData"></param>
	//private MCompressedImage createImage(byte[] imageData)
	//{
	//	uint imageHeight = (uint)renderTexture.height;
	//	uint imageWidth = (uint)renderTexture.width;
	//	MCompressedImage rosImage = new MCompressedImage(new RosMessageTypes.Std.MHeader(), "png", imageData);

	//	//MImage rosImage = new MImage(new RosMessageTypes.Std.MHeader(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
	//	return rosImage;
	//	//PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);
	//	//ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);
		
	//}

	//IEnumerator PublishCamera(float secondsBetweenSpawns)
	//{
	//	while(true)
	//	{
	//		MCompressedImage rosImage = createImage(PoseEstimation());
	//		MAgentState state = new MAgentState();
	//		//rosImage.header.stamp.secs = (uint)Time.realtimeSinceStartup;
	//		//state.header.stamp.secs = (uint)Time.realtimeSinceStartup;
	//		state.grip = 0.66f;
	//		ros.Send(PUB_VISUAL_OBS, rosImage);
	//		ros.Send(PUB_VECTOR_OBS, state);
	//		yield return new WaitForSeconds(secondsBetweenSpawns);
	//	}
	//}
}
