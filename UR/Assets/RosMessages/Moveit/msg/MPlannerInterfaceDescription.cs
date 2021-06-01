//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MPlannerInterfaceDescription : Message
    {
        public const string RosMessageName = "moveit_msgs/PlannerInterfaceDescription";

        //  The name of the planner interface
        public string name;
        //  The names of the planner ids within the interface
        public string[] planner_ids;

        public MPlannerInterfaceDescription()
        {
            this.name = "";
            this.planner_ids = new string[0];
        }

        public MPlannerInterfaceDescription(string name, string[] planner_ids)
        {
            this.name = name;
            this.planner_ids = planner_ids;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            
            listOfSerializations.Add(BitConverter.GetBytes(planner_ids.Length));
            foreach(var entry in planner_ids)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            
            var planner_idsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.planner_ids= new string[planner_idsArrayLength];
            for(var i = 0; i < planner_idsArrayLength; i++)
            {
                var planner_idsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.planner_ids[i] = DeserializeString(data, offset, planner_idsStringBytesLength);
                offset += planner_idsStringBytesLength;
            }

            return offset;
        }

        public override string ToString()
        {
            return "MPlannerInterfaceDescription: " +
            "\nname: " + name.ToString() +
            "\nplanner_ids: " + System.String.Join(", ", planner_ids.ToList());
        }
    }
}