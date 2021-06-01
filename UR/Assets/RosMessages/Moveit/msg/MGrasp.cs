//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MGrasp : Message
    {
        public const string RosMessageName = "moveit_msgs/Grasp";

        //  This message contains a description of a grasp that would be used
        //  with a particular end-effector to grasp an object, including how to
        //  approach it, grip it, etc.  This message does not contain any
        //  information about a "grasp point" (a position ON the object).
        //  Whatever generates this message should have already combined
        //  information about grasp points with information about the geometry
        //  of the end-effector to compute the grasp_pose in this message.
        //  A name for this grasp
        public string id;
        //  The internal posture of the hand for the pre-grasp
        //  only positions are used
        public Trajectory.MJointTrajectory pre_grasp_posture;
        //  The internal posture of the hand for the grasp
        //  positions and efforts are used
        public Trajectory.MJointTrajectory grasp_posture;
        //  The position of the end-effector for the grasp.  This is the pose of
        //  the "parent_link" of the end-effector, not actually the pose of any
        //  link *in* the end-effector.  Typically this would be the pose of the
        //  most distal wrist link before the hand (end-effector) links began.
        public Geometry.MPoseStamped grasp_pose;
        //  The estimated probability of success for this grasp, or some other
        //  measure of how "good" it is.
        public double grasp_quality;
        //  The approach direction to take before picking an object
        public MGripperTranslation pre_grasp_approach;
        //  The retreat direction to take after a grasp has been completed (object is attached)
        public MGripperTranslation post_grasp_retreat;
        //  The retreat motion to perform when releasing the object; this information
        //  is not necessary for the grasp itself, but when releasing the object,
        //  the information will be necessary. The grasp used to perform a pickup
        //  is returned as part of the result, so this information is available for 
        //  later use.
        public MGripperTranslation post_place_retreat;
        //  the maximum contact force to use while grasping (<=0 to disable)
        public float max_contact_force;
        //  an optional list of obstacles that we have semantic information about
        //  and that can be touched/pushed/moved in the course of grasping
        public string[] allowed_touch_objects;

        public MGrasp()
        {
            this.id = "";
            this.pre_grasp_posture = new Trajectory.MJointTrajectory();
            this.grasp_posture = new Trajectory.MJointTrajectory();
            this.grasp_pose = new Geometry.MPoseStamped();
            this.grasp_quality = 0.0;
            this.pre_grasp_approach = new MGripperTranslation();
            this.post_grasp_retreat = new MGripperTranslation();
            this.post_place_retreat = new MGripperTranslation();
            this.max_contact_force = 0.0f;
            this.allowed_touch_objects = new string[0];
        }

        public MGrasp(string id, Trajectory.MJointTrajectory pre_grasp_posture, Trajectory.MJointTrajectory grasp_posture, Geometry.MPoseStamped grasp_pose, double grasp_quality, MGripperTranslation pre_grasp_approach, MGripperTranslation post_grasp_retreat, MGripperTranslation post_place_retreat, float max_contact_force, string[] allowed_touch_objects)
        {
            this.id = id;
            this.pre_grasp_posture = pre_grasp_posture;
            this.grasp_posture = grasp_posture;
            this.grasp_pose = grasp_pose;
            this.grasp_quality = grasp_quality;
            this.pre_grasp_approach = pre_grasp_approach;
            this.post_grasp_retreat = post_grasp_retreat;
            this.post_place_retreat = post_place_retreat;
            this.max_contact_force = max_contact_force;
            this.allowed_touch_objects = allowed_touch_objects;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(pre_grasp_posture.SerializationStatements());
            listOfSerializations.AddRange(grasp_posture.SerializationStatements());
            listOfSerializations.AddRange(grasp_pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.grasp_quality));
            listOfSerializations.AddRange(pre_grasp_approach.SerializationStatements());
            listOfSerializations.AddRange(post_grasp_retreat.SerializationStatements());
            listOfSerializations.AddRange(post_place_retreat.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.max_contact_force));
            
            listOfSerializations.Add(BitConverter.GetBytes(allowed_touch_objects.Length));
            foreach(var entry in allowed_touch_objects)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            offset = this.pre_grasp_posture.Deserialize(data, offset);
            offset = this.grasp_posture.Deserialize(data, offset);
            offset = this.grasp_pose.Deserialize(data, offset);
            this.grasp_quality = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.pre_grasp_approach.Deserialize(data, offset);
            offset = this.post_grasp_retreat.Deserialize(data, offset);
            offset = this.post_place_retreat.Deserialize(data, offset);
            this.max_contact_force = BitConverter.ToSingle(data, offset);
            offset += 4;
            
            var allowed_touch_objectsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.allowed_touch_objects= new string[allowed_touch_objectsArrayLength];
            for(var i = 0; i < allowed_touch_objectsArrayLength; i++)
            {
                var allowed_touch_objectsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.allowed_touch_objects[i] = DeserializeString(data, offset, allowed_touch_objectsStringBytesLength);
                offset += allowed_touch_objectsStringBytesLength;
            }

            return offset;
        }

        public override string ToString()
        {
            return "MGrasp: " +
            "\nid: " + id.ToString() +
            "\npre_grasp_posture: " + pre_grasp_posture.ToString() +
            "\ngrasp_posture: " + grasp_posture.ToString() +
            "\ngrasp_pose: " + grasp_pose.ToString() +
            "\ngrasp_quality: " + grasp_quality.ToString() +
            "\npre_grasp_approach: " + pre_grasp_approach.ToString() +
            "\npost_grasp_retreat: " + post_grasp_retreat.ToString() +
            "\npost_place_retreat: " + post_place_retreat.ToString() +
            "\nmax_contact_force: " + max_contact_force.ToString() +
            "\nallowed_touch_objects: " + System.String.Join(", ", allowed_touch_objects.ToList());
        }
    }
}
