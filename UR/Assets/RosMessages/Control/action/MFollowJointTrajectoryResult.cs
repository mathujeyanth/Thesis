//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    public class MFollowJointTrajectoryResult : Message
    {
        public const string RosMessageName = "control_msgs/FollowJointTrajectory";

        public int error_code;
        public const int SUCCESSFUL = 0;
        public const int INVALID_GOAL = -1;
        public const int INVALID_JOINTS = -2;
        public const int OLD_HEADER_TIMESTAMP = -3;
        public const int PATH_TOLERANCE_VIOLATED = -4;
        public const int GOAL_TOLERANCE_VIOLATED = -5;
        //  Human readable description of the error code. Contains complementary
        //  information that is especially useful when execution fails, for instance:
        //  - INVALID_GOAL: The reason for the invalid goal (e.g., the requested
        //    trajectory is in the past).
        //  - INVALID_JOINTS: The mismatch between the expected controller joints
        //    and those provided in the goal.
        //  - PATH_TOLERANCE_VIOLATED and GOAL_TOLERANCE_VIOLATED: Which joint
        //    violated which tolerance, and by how much.
        public string error_string;

        public MFollowJointTrajectoryResult()
        {
            this.error_code = 0;
            this.error_string = "";
        }

        public MFollowJointTrajectoryResult(int error_code, string error_string)
        {
            this.error_code = error_code;
            this.error_string = error_string;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.error_code));
            listOfSerializations.Add(SerializeString(this.error_string));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.error_code = BitConverter.ToInt32(data, offset);
            offset += 4;
            var error_stringStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.error_string = DeserializeString(data, offset, error_stringStringBytesLength);
            offset += error_stringStringBytesLength;

            return offset;
        }

        public override string ToString()
        {
            return "MFollowJointTrajectoryResult: " +
            "\nerror_code: " + error_code.ToString() +
            "\nerror_string: " + error_string.ToString();
        }
    }
}
