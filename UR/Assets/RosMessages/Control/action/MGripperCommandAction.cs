using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Control
{
    public class MGripperCommandAction : Action<MGripperCommandActionGoal, MGripperCommandActionResult, MGripperCommandActionFeedback, MGripperCommandGoal, MGripperCommandResult, MGripperCommandFeedback>
    {
        public const string RosMessageName = "control_msgs/GripperCommandAction";

        public MGripperCommandAction() : base()
        {
            this.action_goal = new MGripperCommandActionGoal();
            this.action_result = new MGripperCommandActionResult();
            this.action_feedback = new MGripperCommandActionFeedback();
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
