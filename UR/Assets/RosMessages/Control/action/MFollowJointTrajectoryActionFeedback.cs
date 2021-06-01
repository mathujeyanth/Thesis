using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Control
{
    public class MFollowJointTrajectoryActionFeedback : ActionFeedback<MFollowJointTrajectoryFeedback>
    {
        public const string RosMessageName = "control_msgs/FollowJointTrajectoryActionFeedback";

        public MFollowJointTrajectoryActionFeedback() : base()
        {
            this.feedback = new MFollowJointTrajectoryFeedback();
        }

        public MFollowJointTrajectoryActionFeedback(MHeader header, MGoalStatus status, MFollowJointTrajectoryFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.status.SerializationStatements());
            listOfSerializations.AddRange(this.feedback.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.status.Deserialize(data, offset);
            offset = this.feedback.Deserialize(data, offset);

            return offset;
        }

    }
}
