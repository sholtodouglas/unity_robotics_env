//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MRPYState : Message
    {
        public const string RosMessageName = "robotics_demo/RPYState";

        public MRPYProprioState proprio;
        public MAchievedGoal ag;

        public MRPYState()
        {
            this.proprio = new MRPYProprioState();
            this.ag = new MAchievedGoal();
        }

        public MRPYState(MRPYProprioState proprio, MAchievedGoal ag)
        {
            this.proprio = proprio;
            this.ag = ag;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(proprio.SerializationStatements());
            listOfSerializations.AddRange(ag.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.proprio.Deserialize(data, offset);
            offset = this.ag.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MRPYState: " +
            "\nproprio: " + proprio.ToString() +
            "\nag: " + ag.ToString();
        }
    }
}
