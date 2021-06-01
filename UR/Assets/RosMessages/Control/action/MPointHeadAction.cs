using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Control
{
    public class MPointHeadAction : Action<MPointHeadActionGoal, MPointHeadActionResult, MPointHeadActionFeedback, MPointHeadGoal, MPointHeadResult, MPointHeadFeedback>
    {
        public const string RosMessageName = "control_msgs/PointHeadAction";

        public MPointHeadAction() : base()
        {
            this.action_goal = new MPointHeadActionGoal();
            this.action_result = new MPointHeadActionResult();
            this.action_feedback = new MPointHeadActionFeedback();
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
