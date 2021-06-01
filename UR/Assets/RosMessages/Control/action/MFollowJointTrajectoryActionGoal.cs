using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Control
{
    public class MFollowJointTrajectoryActionGoal : ActionGoal<MFollowJointTrajectoryGoal>
    {
        public const string RosMessageName = "control_msgs/FollowJointTrajectoryActionGoal";

        public MFollowJointTrajectoryActionGoal() : base()
        {
            this.goal = new MFollowJointTrajectoryGoal();
        }

        public MFollowJointTrajectoryActionGoal(MHeader header, MGoalID goal_id, MFollowJointTrajectoryGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.goal_id.SerializationStatements());
            listOfSerializations.AddRange(this.goal.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.goal_id.Deserialize(data, offset);
            offset = this.goal.Deserialize(data, offset);

            return offset;
        }

    }
}
