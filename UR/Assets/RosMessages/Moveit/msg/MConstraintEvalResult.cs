//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MConstraintEvalResult : Message
    {
        public const string RosMessageName = "moveit_msgs/ConstraintEvalResult";

        //  This message contains result from constraint evaluation
        //  result specifies the result of constraint evaluation 
        //  (true indicates state satisfies constraint, false indicates state violates constraint)
        //  if false, distance specifies a measure of the distance of the state from the constraint
        //  if true, distance is set to zero
        public bool result;
        public double distance;

        public MConstraintEvalResult()
        {
            this.result = false;
            this.distance = 0.0;
        }

        public MConstraintEvalResult(bool result, double distance)
        {
            this.result = result;
            this.distance = distance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.result));
            listOfSerializations.Add(BitConverter.GetBytes(this.distance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.result = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.distance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MConstraintEvalResult: " +
            "\nresult: " + result.ToString() +
            "\ndistance: " + distance.ToString();
        }
    }
}
