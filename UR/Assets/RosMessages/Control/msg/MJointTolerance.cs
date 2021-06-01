//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    public class MJointTolerance : Message
    {
        public const string RosMessageName = "control_msgs/JointTolerance";

        //  The tolerances specify the amount the position, velocity, and
        //  accelerations can vary from the setpoints.  For example, in the case
        //  of trajectory control, when the actual position varies beyond
        //  (desired position + position tolerance), the trajectory goal may
        //  abort.
        //  
        //  There are two special values for tolerances:
        //   * 0 - The tolerance is unspecified and will remain at whatever the default is
        //   * -1 - The tolerance is "erased".  If there was a default, the joint will be
        //          allowed to move without restriction.
        public string name;
        public double position;
        //  in radians or meters (for a revolute or prismatic joint, respectively)
        public double velocity;
        //  in rad/sec or m/sec
        public double acceleration;
        //  in rad/sec^2 or m/sec^2

        public MJointTolerance()
        {
            this.name = "";
            this.position = 0.0;
            this.velocity = 0.0;
            this.acceleration = 0.0;
        }

        public MJointTolerance(string name, double position, double velocity, double acceleration)
        {
            this.name = name;
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(BitConverter.GetBytes(this.position));
            listOfSerializations.Add(BitConverter.GetBytes(this.velocity));
            listOfSerializations.Add(BitConverter.GetBytes(this.acceleration));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            this.position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.velocity = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.acceleration = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MJointTolerance: " +
            "\nname: " + name.ToString() +
            "\nposition: " + position.ToString() +
            "\nvelocity: " + velocity.ToString() +
            "\nacceleration: " + acceleration.ToString();
        }
    }
}
