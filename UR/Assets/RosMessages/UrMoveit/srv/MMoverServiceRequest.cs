//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UrMoveit
{
    public class MMoverServiceRequest : Message
    {
        public const string RosMessageName = "ur_moveit/MoverService";

        public MURMoveitJoints joints_input;
        public Geometry.MPose pick_pose;
        public Geometry.MPose place_pose;

        public MMoverServiceRequest()
        {
            this.joints_input = new MURMoveitJoints();
            this.pick_pose = new Geometry.MPose();
            this.place_pose = new Geometry.MPose();
        }

        public MMoverServiceRequest(MURMoveitJoints joints_input, Geometry.MPose pick_pose, Geometry.MPose place_pose)
        {
            this.joints_input = joints_input;
            this.pick_pose = pick_pose;
            this.place_pose = place_pose;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(joints_input.SerializationStatements());
            listOfSerializations.AddRange(pick_pose.SerializationStatements());
            listOfSerializations.AddRange(place_pose.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.joints_input.Deserialize(data, offset);
            offset = this.pick_pose.Deserialize(data, offset);
            offset = this.place_pose.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MMoverServiceRequest: " +
            "\njoints_input: " + joints_input.ToString() +
            "\npick_pose: " + pick_pose.ToString() +
            "\nplace_pose: " + place_pose.ToString();
        }
    }
}
