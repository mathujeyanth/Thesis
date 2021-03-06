//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MPositionServiceResponse : Message
    {
        public const string RosMessageName = "robotics_demo/PositionService";

        public MPosRot output;

        public MPositionServiceResponse()
        {
            this.output = new MPosRot();
        }

        public MPositionServiceResponse(MPosRot output)
        {
            this.output = output;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(output.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.output.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MPositionServiceResponse: " +
            "\noutput: " + output.ToString();
        }
    }
}
