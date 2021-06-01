using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Control
{
    public class MJointTrajectoryActionResult : ActionResult<MJointTrajectoryResult>
    {
        public const string RosMessageName = "control_msgs/JointTrajectoryActionResult";

        public MJointTrajectoryActionResult() : base()
        {
            this.result = new MJointTrajectoryResult();
        }

        public MJointTrajectoryActionResult(MHeader header, MGoalStatus status, MJointTrajectoryResult result) : base(header, status)
        {
            this.result = result;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.status.SerializationStatements());
            listOfSerializations.AddRange(this.result.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.status.Deserialize(data, offset);
            offset = this.result.Deserialize(data, offset);

            return offset;
        }

    }
}
