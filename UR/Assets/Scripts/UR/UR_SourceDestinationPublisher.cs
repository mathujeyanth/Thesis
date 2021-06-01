using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.UrMoveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;


public class UR_SourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    
    // Variables required for ROS communication
    public string topicName = "UR_SourceDestination_input";

    public GameObject UR5E;
    public GameObject target;
    public GameObject targetPlacement;
    

    private int numRobotJoints = 6;
    private readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);
    
    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    
    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = UR5E.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string upper_arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = UR5E.transform.Find(upper_arm_link).GetComponent<ArticulationBody>();
        
        string forearm_link = upper_arm_link + "/forearm_link";
        jointArticulationBodies[2] = UR5E.transform.Find(forearm_link).GetComponent<ArticulationBody>();
        
        string wrist_1_link = forearm_link + "/wrist_1_link";
        jointArticulationBodies[3] = UR5E.transform.Find(wrist_1_link).GetComponent<ArticulationBody>();
        
        string wrist_2_link = wrist_1_link + "/wrist_2_link";
        jointArticulationBodies[4] = UR5E.transform.Find(wrist_2_link).GetComponent<ArticulationBody>();
        
        string wrist_3_link = wrist_2_link + "/wrist_3_link";
        jointArticulationBodies[5] = UR5E.transform.Find(wrist_3_link).GetComponent<ArticulationBody>();
    }

    public void Publish()
    {
        // RobotStateRTMsg sourceDestinationMessage = new RobotStateRTMsg();
        // sourceDestinationMessage.time = 0.9;
        // double[] q_target = new double[numRobotJoints];
		// for (int i = 0; i < numRobotJoints; i++) {
		// 	q_target[i] = jointArticulationBodies[i].xDrive.target;
		// }
        // sourceDestinationMessage.q_actual = q_target;

        MURMoveitJoints sourceDestinationMessage = new MURMoveitJoints();

        sourceDestinationMessage.joint_00 = jointArticulationBodies[0].xDrive.target;
        sourceDestinationMessage.joint_01 = jointArticulationBodies[1].xDrive.target;
        sourceDestinationMessage.joint_02 = jointArticulationBodies[2].xDrive.target;
        sourceDestinationMessage.joint_03 = jointArticulationBodies[3].xDrive.target;
        sourceDestinationMessage.joint_04 = jointArticulationBodies[4].xDrive.target;
        sourceDestinationMessage.joint_05 = jointArticulationBodies[5].xDrive.target;

        // Pick Pose
        sourceDestinationMessage.pick_pose = new RosMessageTypes.Geometry.MPose
        {
            position = target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new RosMessageTypes.Geometry.MPose
        {
            position = targetPlacement.transform.position.To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        //double[] qd_target;
        //double[] qdd_target;
        //double[] i_target;
        //double[] m_target;
        //double[] q_actual;
        //double[] qd_actual;
        //double[] i_actual;
        //double[] tool_acc_values;
        //double[] tcp_force;
        //double[] tool_vector;
        //double[] tcp_speed;
        //double digital_input_bits;
        //double[] motor_temperatures;
        //double controller_timer;
        //double test_value;
        //double robot_mode;
        //double[] joint_modes;
        ros.Send(topicName,sourceDestinationMessage);
    }
}
