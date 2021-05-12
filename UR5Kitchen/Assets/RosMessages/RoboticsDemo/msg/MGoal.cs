//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MGoal : Message
    {
        public const string RosMessageName = "robotics_demo/Goal";

        public Std.MString type;
        public MAchievedGoal state_goal;
        public Sensor.MImage img_goal;
        public Std.MString lang_goal;

        public MGoal()
        {
            this.type = new Std.MString();
            this.state_goal = new MAchievedGoal();
            this.img_goal = new Sensor.MImage();
            this.lang_goal = new Std.MString();
        }

        public MGoal(Std.MString type, MAchievedGoal state_goal, Sensor.MImage img_goal, Std.MString lang_goal)
        {
            this.type = type;
            this.state_goal = state_goal;
            this.img_goal = img_goal;
            this.lang_goal = lang_goal;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(type.SerializationStatements());
            listOfSerializations.AddRange(state_goal.SerializationStatements());
            listOfSerializations.AddRange(img_goal.SerializationStatements());
            listOfSerializations.AddRange(lang_goal.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.type.Deserialize(data, offset);
            offset = this.state_goal.Deserialize(data, offset);
            offset = this.img_goal.Deserialize(data, offset);
            offset = this.lang_goal.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MGoal: " +
            "\ntype: " + type.ToString() +
            "\nstate_goal: " + state_goal.ToString() +
            "\nimg_goal: " + img_goal.ToString() +
            "\nlang_goal: " + lang_goal.ToString();
        }
    }
}