using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Control
{
    public class MFollowJointTrajectoryAction : Action<MFollowJointTrajectoryActionGoal, MFollowJointTrajectoryActionResult, MFollowJointTrajectoryActionFeedback, MFollowJointTrajectoryGoal, MFollowJointTrajectoryResult, MFollowJointTrajectoryFeedback>
    {
        public const string RosMessageName = "control_msgs/FollowJointTrajectoryAction";

        public MFollowJointTrajectoryAction() : base()
        {
            this.action_goal = new MFollowJointTrajectoryActionGoal();
            this.action_result = new MFollowJointTrajectoryActionResult();
            this.action_feedback = new MFollowJointTrajectoryActionFeedback();
        }

        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.action_goal.SerializationStatements());
            listOfSerializations.AddRange(this.action_result.SerializationStatements());
            listOfSerializations.AddRange(this.action_feedback.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.action_goal.Deserialize(data, offset);
            offset = this.action_result.Deserialize(data, offset);
            offset = this.action_feedback.Deserialize(data, offset);

            return offset;
        }

    }
}
