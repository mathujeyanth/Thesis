//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    public class MGripperCommand : Message
    {
        public const string RosMessageName = "control_msgs/GripperCommand";

        public double position;
        public double max_effort;

        public MGripperCommand()
        {
            this.position = 0.0;
            this.max_effort = 0.0;
        }

        public MGripperCommand(double position, double max_effort)
        {
            this.position = position;
            this.max_effort = max_effort;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.position));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_effort));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.max_effort = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MGripperCommand: " +
            "\nposition: " + position.ToString() +
            "\nmax_effort: " + max_effort.ToString();
        }
    }
}