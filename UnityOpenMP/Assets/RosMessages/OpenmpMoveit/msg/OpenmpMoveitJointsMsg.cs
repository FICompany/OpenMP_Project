//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.OpenmpMoveit
{
    [Serializable]
    public class OpenmpMoveitJointsMsg : Message
    {
        public const string k_RosMessageName = "openmp_moveit/OpenmpMoveitJoints";
        public override string RosMessageName => k_RosMessageName;

        public double[] joints;
        public Geometry.PoseMsg pick_pose;
        public Geometry.PoseMsg place_pose;

        public OpenmpMoveitJointsMsg()
        {
            this.joints = new double[6];
            this.pick_pose = new Geometry.PoseMsg();
            this.place_pose = new Geometry.PoseMsg();
        }

        public OpenmpMoveitJointsMsg(double[] joints, Geometry.PoseMsg pick_pose, Geometry.PoseMsg place_pose)
        {
            this.joints = joints;
            this.pick_pose = pick_pose;
            this.place_pose = place_pose;
        }

        public static OpenmpMoveitJointsMsg Deserialize(MessageDeserializer deserializer) => new OpenmpMoveitJointsMsg(deserializer);

        private OpenmpMoveitJointsMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.joints, sizeof(double), 6);
            this.pick_pose = Geometry.PoseMsg.Deserialize(deserializer);
            this.place_pose = Geometry.PoseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.joints);
            serializer.Write(this.pick_pose);
            serializer.Write(this.place_pose);
        }

        public override string ToString()
        {
            return "OpenmpMoveitJointsMsg: " +
            "\njoints: " + System.String.Join(", ", joints.ToList()) +
            "\npick_pose: " + pick_pose.ToString() +
            "\nplace_pose: " + place_pose.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
