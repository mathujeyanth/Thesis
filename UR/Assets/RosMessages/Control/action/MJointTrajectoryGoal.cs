//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    public class MJointTrajectoryGoal : Message
    {
        public const string RosMessageName = "control_msgs/JointTrajectory";

        public Trajectory.MJointTrajectory trajectory;

        public MJointTrajectoryGoal()
        {
            this.trajectory = new Trajectory.MJointTrajectory();
        }

        public MJointTrajectoryGoal(Trajectory.MJointTrajectory trajectory)
        {
            this.trajectory = trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(trajectory.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.trajectory.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MJointTrajectoryGoal: " +
            "\ntrajectory: " + trajectory.ToString();
        }
    }
}
