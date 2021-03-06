//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UrMoveit
{
    public class MPoseEstimationServiceResponse : Message
    {
        public const string RosMessageName = "ur_moveit/PoseEstimationService";

        public Geometry.MPose estimated_pose;

        public MPoseEstimationServiceResponse()
        {
            this.estimated_pose = new Geometry.MPose();
        }

        public MPoseEstimationServiceResponse(Geometry.MPose estimated_pose)
        {
            this.estimated_pose = estimated_pose;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(estimated_pose.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.estimated_pose.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MPoseEstimationServiceResponse: " +
            "\nestimated_pose: " + estimated_pose.ToString();
        }
    }
}
