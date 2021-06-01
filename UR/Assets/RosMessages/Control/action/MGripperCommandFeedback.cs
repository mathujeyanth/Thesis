//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    public class MGripperCommandFeedback : Message
    {
        public const string RosMessageName = "control_msgs/GripperCommand";

        public double position;
        //  The current gripper gap size (in meters)
        public double effort;
        //  The current effort exerted (in Newtons)
        public bool stalled;
        //  True iff the gripper is exerting max effort and not moving
        public bool reached_goal;
        //  True iff the gripper position has reached the commanded setpoint

        public MGripperCommandFeedback()
        {
            this.position = 0.0;
            this.effort = 0.0;
            this.stalled = false;
            this.reached_goal = false;
        }

        public MGripperCommandFeedback(double position, double effort, bool stalled, bool reached_goal)
        {
            this.position = position;
            this.effort = effort;
            this.stalled = stalled;
            this.reached_goal = reached_goal;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.position));
            listOfSerializations.Add(BitConverter.GetBytes(this.effort));
            listOfSerializations.Add(BitConverter.GetBytes(this.stalled));
            listOfSerializations.Add(BitConverter.GetBytes(this.reached_goal));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.effort = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.stalled = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.reached_goal = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MGripperCommandFeedback: " +
            "\nposition: " + position.ToString() +
            "\neffort: " + effort.ToString() +
            "\nstalled: " + stalled.ToString() +
            "\nreached_goal: " + reached_goal.ToString();
        }
    }
}