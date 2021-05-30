//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MresetResponse : Message
    {
        public const string RosMessageName = "robotics_demo/reset";

        public MObservation state;

        public MresetResponse()
        {
            this.state = new MObservation();
        }

        public MresetResponse(MObservation state)
        {
            this.state = state;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(state.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.state.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MresetResponse: " +
            "\nstate: " + state.ToString();
        }
    }
}
