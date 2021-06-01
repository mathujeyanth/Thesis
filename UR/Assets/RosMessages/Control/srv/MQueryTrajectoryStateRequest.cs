//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Control
{
    public class MQueryTrajectoryStateRequest : Message
    {
        public const string RosMessageName = "control_msgs/QueryTrajectoryState";

        public MTime time;

        public MQueryTrajectoryStateRequest()
        {
            this.time = new MTime();
        }

        public MQueryTrajectoryStateRequest(MTime time)
        {
            this.time = time;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(time.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.time.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MQueryTrajectoryStateRequest: " +
            "\ntime: " + time.ToString();
        }
    }
}