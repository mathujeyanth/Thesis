//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UrMoveit
{
    public class MPoseEstimationServiceRequest : Message
    {
        public const string RosMessageName = "ur_moveit/PoseEstimationService";

        public Sensor.MImage image;

        public MPoseEstimationServiceRequest()
        {
            this.image = new Sensor.MImage();
        }

        public MPoseEstimationServiceRequest(Sensor.MImage image)
        {
            this.image = image;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(image.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.image.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MPoseEstimationServiceRequest: " +
            "\nimage: " + image.ToString();
        }
    }
}
