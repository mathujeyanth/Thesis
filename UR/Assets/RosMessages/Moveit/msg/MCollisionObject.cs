//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    public class MCollisionObject : Message
    {
        public const string RosMessageName = "moveit_msgs/CollisionObject";

        //  A header, used for interpreting the poses
        public MHeader header;
        //  DISCLAIMER: This field is not in use yet and all other poses
        //  are still interpreted in the header frame.
        //  https://github.com/ros-planning/moveit/pull/2037
        //  implements the actual logic for this field.
        //  ---
        //  The object's pose relative to the header frame.
        //  The shapes and subframe poses are defined relative to this pose.
        public Geometry.MPose pose;
        //  The id of the object (name used in MoveIt)
        public string id;
        //  The object type in a database of known objects
        public ObjectRecognition.MObjectType type;
        //  The collision geometries associated with the object.
        //  Their poses are with respect to the object's pose
        //  Solid geometric primitives
        public Shape.MSolidPrimitive[] primitives;
        public Geometry.MPose[] primitive_poses;
        //  Meshes
        public Shape.MMesh[] meshes;
        public Geometry.MPose[] mesh_poses;
        //  Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
        public Shape.MPlane[] planes;
        public Geometry.MPose[] plane_poses;
        //  Named subframes on the object. Use these to define points of interest on the object that you want
        //  to plan with (e.g. "tip", "spout", "handle"). The id of the object will be prepended to the subframe.
        //  If an object with the id "screwdriver" and a subframe "tip" is in the scene, you can use the frame
        //  "screwdriver/tip" for planning.
        //  The length of the subframe_names and subframe_poses has to be identical.
        public string[] subframe_names;
        public Geometry.MPose[] subframe_poses;
        //  Adds the object to the planning scene. If the object previously existed, it is replaced.
        public const sbyte ADD = 0;
        //  Removes the object from the environment entirely (everything that matches the specified id)
        public const sbyte REMOVE = 1;
        //  Append to an object that already exists in the planning scene. If the object does not exist, it is added.
        public const sbyte APPEND = 2;
        //  If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
        //  if solely moving the object is desired
        public const sbyte MOVE = 3;
        //  Operation to be performed
        public sbyte operation;

        public MCollisionObject()
        {
            this.header = new MHeader();
            this.pose = new Geometry.MPose();
            this.id = "";
            this.type = new ObjectRecognition.MObjectType();
            this.primitives = new Shape.MSolidPrimitive[0];
            this.primitive_poses = new Geometry.MPose[0];
            this.meshes = new Shape.MMesh[0];
            this.mesh_poses = new Geometry.MPose[0];
            this.planes = new Shape.MPlane[0];
            this.plane_poses = new Geometry.MPose[0];
            this.subframe_names = new string[0];
            this.subframe_poses = new Geometry.MPose[0];
            this.operation = 0;
        }

        public MCollisionObject(MHeader header, Geometry.MPose pose, string id, ObjectRecognition.MObjectType type, Shape.MSolidPrimitive[] primitives, Geometry.MPose[] primitive_poses, Shape.MMesh[] meshes, Geometry.MPose[] mesh_poses, Shape.MPlane[] planes, Geometry.MPose[] plane_poses, string[] subframe_names, Geometry.MPose[] subframe_poses, sbyte operation)
        {
            this.header = header;
            this.pose = pose;
            this.id = id;
            this.type = type;
            this.primitives = primitives;
            this.primitive_poses = primitive_poses;
            this.meshes = meshes;
            this.mesh_poses = mesh_poses;
            this.planes = planes;
            this.plane_poses = plane_poses;
            this.subframe_names = subframe_names;
            this.subframe_poses = subframe_poses;
            this.operation = operation;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(type.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(primitives.Length));
            foreach(var entry in primitives)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(primitive_poses.Length));
            foreach(var entry in primitive_poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(meshes.Length));
            foreach(var entry in meshes)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(mesh_poses.Length));
            foreach(var entry in mesh_poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(planes.Length));
            foreach(var entry in planes)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(plane_poses.Length));
            foreach(var entry in plane_poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(subframe_names.Length));
            foreach(var entry in subframe_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(subframe_poses.Length));
            foreach(var entry in subframe_poses)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(new[]{(byte)this.operation});

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            offset = this.type.Deserialize(data, offset);
            
            var primitivesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.primitives= new Shape.MSolidPrimitive[primitivesArrayLength];
            for(var i = 0; i < primitivesArrayLength; i++)
            {
                this.primitives[i] = new Shape.MSolidPrimitive();
                offset = this.primitives[i].Deserialize(data, offset);
            }
            
            var primitive_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.primitive_poses= new Geometry.MPose[primitive_posesArrayLength];
            for(var i = 0; i < primitive_posesArrayLength; i++)
            {
                this.primitive_poses[i] = new Geometry.MPose();
                offset = this.primitive_poses[i].Deserialize(data, offset);
            }
            
            var meshesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.meshes= new Shape.MMesh[meshesArrayLength];
            for(var i = 0; i < meshesArrayLength; i++)
            {
                this.meshes[i] = new Shape.MMesh();
                offset = this.meshes[i].Deserialize(data, offset);
            }
            
            var mesh_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.mesh_poses= new Geometry.MPose[mesh_posesArrayLength];
            for(var i = 0; i < mesh_posesArrayLength; i++)
            {
                this.mesh_poses[i] = new Geometry.MPose();
                offset = this.mesh_poses[i].Deserialize(data, offset);
            }
            
            var planesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.planes= new Shape.MPlane[planesArrayLength];
            for(var i = 0; i < planesArrayLength; i++)
            {
                this.planes[i] = new Shape.MPlane();
                offset = this.planes[i].Deserialize(data, offset);
            }
            
            var plane_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.plane_poses= new Geometry.MPose[plane_posesArrayLength];
            for(var i = 0; i < plane_posesArrayLength; i++)
            {
                this.plane_poses[i] = new Geometry.MPose();
                offset = this.plane_poses[i].Deserialize(data, offset);
            }
            
            var subframe_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.subframe_names= new string[subframe_namesArrayLength];
            for(var i = 0; i < subframe_namesArrayLength; i++)
            {
                var subframe_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.subframe_names[i] = DeserializeString(data, offset, subframe_namesStringBytesLength);
                offset += subframe_namesStringBytesLength;
            }
            
            var subframe_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.subframe_poses= new Geometry.MPose[subframe_posesArrayLength];
            for(var i = 0; i < subframe_posesArrayLength; i++)
            {
                this.subframe_poses[i] = new Geometry.MPose();
                offset = this.subframe_poses[i].Deserialize(data, offset);
            }
            this.operation = (sbyte)data[offset];;
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MCollisionObject: " +
            "\nheader: " + header.ToString() +
            "\npose: " + pose.ToString() +
            "\nid: " + id.ToString() +
            "\ntype: " + type.ToString() +
            "\nprimitives: " + System.String.Join(", ", primitives.ToList()) +
            "\nprimitive_poses: " + System.String.Join(", ", primitive_poses.ToList()) +
            "\nmeshes: " + System.String.Join(", ", meshes.ToList()) +
            "\nmesh_poses: " + System.String.Join(", ", mesh_poses.ToList()) +
            "\nplanes: " + System.String.Join(", ", planes.ToList()) +
            "\nplane_poses: " + System.String.Join(", ", plane_poses.ToList()) +
            "\nsubframe_names: " + System.String.Join(", ", subframe_names.ToList()) +
            "\nsubframe_poses: " + System.String.Join(", ", subframe_poses.ToList()) +
            "\noperation: " + operation.ToString();
        }
    }
}
