//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    public class MInt32 : Message
    {
        public const string RosMessageName = "std_msgs/Int32";

        public int data;

        public MInt32()
        {
            this.data = 0;
        }

        public MInt32(int data)
        {
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.data));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.data = BitConverter.ToInt32(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MInt32: " +
            "\ndata: " + data.ToString();
        }
    }
}
