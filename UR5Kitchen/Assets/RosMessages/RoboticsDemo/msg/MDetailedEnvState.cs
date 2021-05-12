//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MDetailedEnvState : Message
    {
        public const string RosMessageName = "robotics_demo/DetailedEnvState";

        public float obj1_pos_x;
        public float obj1_pos_y;
        public float obj1_pos_z;
        public float obj1_q1;
        public float obj1_q2;
        public float obj1_q3;
        public float obj1_q4;
        public float button1;
        public float button2;
        public float button3;
        public float drawer;
        public float door;
        public float obj2_pos_x;
        public float obj2_pos_y;
        public float obj2_pos_z;
        public float obj2_q1;
        public float obj2_q2;
        public float obj2_q3;
        public float obj2_q4;
        public float button1_x;
        public float button1_y;
        public float button1_z;
        public float button2_x;
        public float button2_y;
        public float button2_z;
        public float button3_x;
        public float button3_y;
        public float button3_z;

        public MDetailedEnvState()
        {
            this.obj1_pos_x = 0.0f;
            this.obj1_pos_y = 0.0f;
            this.obj1_pos_z = 0.0f;
            this.obj1_q1 = 0.0f;
            this.obj1_q2 = 0.0f;
            this.obj1_q3 = 0.0f;
            this.obj1_q4 = 0.0f;
            this.button1 = 0.0f;
            this.button2 = 0.0f;
            this.button3 = 0.0f;
            this.drawer = 0.0f;
            this.door = 0.0f;
            this.obj2_pos_x = 0.0f;
            this.obj2_pos_y = 0.0f;
            this.obj2_pos_z = 0.0f;
            this.obj2_q1 = 0.0f;
            this.obj2_q2 = 0.0f;
            this.obj2_q3 = 0.0f;
            this.obj2_q4 = 0.0f;
            this.button1_x = 0.0f;
            this.button1_y = 0.0f;
            this.button1_z = 0.0f;
            this.button2_x = 0.0f;
            this.button2_y = 0.0f;
            this.button2_z = 0.0f;
            this.button3_x = 0.0f;
            this.button3_y = 0.0f;
            this.button3_z = 0.0f;
        }

        public MDetailedEnvState(float obj1_pos_x, float obj1_pos_y, float obj1_pos_z, float obj1_q1, float obj1_q2, float obj1_q3, float obj1_q4, float button1, float button2, float button3, float drawer, float door, float obj2_pos_x, float obj2_pos_y, float obj2_pos_z, float obj2_q1, float obj2_q2, float obj2_q3, float obj2_q4, float button1_x, float button1_y, float button1_z, float button2_x, float button2_y, float button2_z, float button3_x, float button3_y, float button3_z)
        {
            this.obj1_pos_x = obj1_pos_x;
            this.obj1_pos_y = obj1_pos_y;
            this.obj1_pos_z = obj1_pos_z;
            this.obj1_q1 = obj1_q1;
            this.obj1_q2 = obj1_q2;
            this.obj1_q3 = obj1_q3;
            this.obj1_q4 = obj1_q4;
            this.button1 = button1;
            this.button2 = button2;
            this.button3 = button3;
            this.drawer = drawer;
            this.door = door;
            this.obj2_pos_x = obj2_pos_x;
            this.obj2_pos_y = obj2_pos_y;
            this.obj2_pos_z = obj2_pos_z;
            this.obj2_q1 = obj2_q1;
            this.obj2_q2 = obj2_q2;
            this.obj2_q3 = obj2_q3;
            this.obj2_q4 = obj2_q4;
            this.button1_x = button1_x;
            this.button1_y = button1_y;
            this.button1_z = button1_z;
            this.button2_x = button2_x;
            this.button2_y = button2_y;
            this.button2_z = button2_z;
            this.button3_x = button3_x;
            this.button3_y = button3_y;
            this.button3_z = button3_z;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_pos_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_pos_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_pos_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_q1));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_q2));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_q3));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj1_q4));
            listOfSerializations.Add(BitConverter.GetBytes(this.button1));
            listOfSerializations.Add(BitConverter.GetBytes(this.button2));
            listOfSerializations.Add(BitConverter.GetBytes(this.button3));
            listOfSerializations.Add(BitConverter.GetBytes(this.drawer));
            listOfSerializations.Add(BitConverter.GetBytes(this.door));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_pos_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_pos_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_pos_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_q1));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_q2));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_q3));
            listOfSerializations.Add(BitConverter.GetBytes(this.obj2_q4));
            listOfSerializations.Add(BitConverter.GetBytes(this.button1_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.button1_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.button1_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.button2_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.button2_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.button2_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.button3_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.button3_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.button3_z));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.obj1_pos_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_pos_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_pos_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_q1 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_q2 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_q3 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj1_q4 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button1 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button2 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button3 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.drawer = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.door = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_pos_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_pos_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_pos_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_q1 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_q2 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_q3 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.obj2_q4 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button1_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button1_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button1_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button2_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button2_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button2_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button3_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button3_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.button3_z = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MDetailedEnvState: " +
            "\nobj1_pos_x: " + obj1_pos_x.ToString() +
            "\nobj1_pos_y: " + obj1_pos_y.ToString() +
            "\nobj1_pos_z: " + obj1_pos_z.ToString() +
            "\nobj1_q1: " + obj1_q1.ToString() +
            "\nobj1_q2: " + obj1_q2.ToString() +
            "\nobj1_q3: " + obj1_q3.ToString() +
            "\nobj1_q4: " + obj1_q4.ToString() +
            "\nbutton1: " + button1.ToString() +
            "\nbutton2: " + button2.ToString() +
            "\nbutton3: " + button3.ToString() +
            "\ndrawer: " + drawer.ToString() +
            "\ndoor: " + door.ToString() +
            "\nobj2_pos_x: " + obj2_pos_x.ToString() +
            "\nobj2_pos_y: " + obj2_pos_y.ToString() +
            "\nobj2_pos_z: " + obj2_pos_z.ToString() +
            "\nobj2_q1: " + obj2_q1.ToString() +
            "\nobj2_q2: " + obj2_q2.ToString() +
            "\nobj2_q3: " + obj2_q3.ToString() +
            "\nobj2_q4: " + obj2_q4.ToString() +
            "\nbutton1_x: " + button1_x.ToString() +
            "\nbutton1_y: " + button1_y.ToString() +
            "\nbutton1_z: " + button1_z.ToString() +
            "\nbutton2_x: " + button2_x.ToString() +
            "\nbutton2_y: " + button2_y.ToString() +
            "\nbutton2_z: " + button2_z.ToString() +
            "\nbutton3_x: " + button3_x.ToString() +
            "\nbutton3_y: " + button3_y.ToString() +
            "\nbutton3_z: " + button3_z.ToString();
        }
    }
}
