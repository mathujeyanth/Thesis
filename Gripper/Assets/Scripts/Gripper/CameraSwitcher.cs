using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraSwitcher : MonoBehaviour
{
    public Camera wristCamera;
    public Camera fullCamera;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha0) && fullCamera.targetDisplay == 0){
            fullCamera.targetDisplay = 1;
            wristCamera.targetDisplay = 0;
        }   
        else if (Input.GetKeyDown(KeyCode.Alpha0) && fullCamera.targetDisplay == 1){
            fullCamera.targetDisplay = 0;
            wristCamera.targetDisplay = 1;
        }   
    }
}
