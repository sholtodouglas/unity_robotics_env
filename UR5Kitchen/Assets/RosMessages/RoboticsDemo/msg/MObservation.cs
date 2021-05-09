//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MObservation : Message
    {
        public const string RosMessageName = "robotics_demo/Observation";

        public MQuaternionProprioState proprio;
        public MAchievedGoal ag;
        public MVelocities vels;
        public Sensor.MImage shoulderImage;
        public Sensor.MImage gripperImage;
        public byte[] imq2;
        public byte[] imq3;
        public byte[] imq4;
        public double time;

        public MObservation()
        {
            this.proprio = new MQuaternionProprioState();
            this.ag = new MAchievedGoal();
            this.vels = new MVelocities();
            this.shoulderImage = new Sensor.MImage();
            this.gripperImage = new Sensor.MImage();
            this.imq2 = new byte[0];
            this.imq3 = new byte[0];
            this.imq4 = new byte[0];
            this.time = 0.0;
        }

        public MObservation(MQuaternionProprioState proprio, MAchievedGoal ag, MVelocities vels, Sensor.MImage shoulderImage, Sensor.MImage gripperImage, byte[] imq2, byte[] imq3, byte[] imq4, double time)
        {
            this.proprio = proprio;
            this.ag = ag;
            this.vels = vels;
            this.shoulderImage = shoulderImage;
            this.gripperImage = gripperImage;
            this.imq2 = imq2;
            this.imq3 = imq3;
            this.imq4 = imq4;
            this.time = time;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(proprio.SerializationStatements());
            listOfSerializations.AddRange(ag.SerializationStatements());
            listOfSerializations.AddRange(vels.SerializationStatements());
            listOfSerializations.AddRange(shoulderImage.SerializationStatements());
            listOfSerializations.AddRange(gripperImage.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(imq2.Length));
            listOfSerializations.Add(this.imq2);
            
            listOfSerializations.Add(BitConverter.GetBytes(imq3.Length));
            listOfSerializations.Add(this.imq3);
            
            listOfSerializations.Add(BitConverter.GetBytes(imq4.Length));
            listOfSerializations.Add(this.imq4);
            listOfSerializations.Add(BitConverter.GetBytes(this.time));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.proprio.Deserialize(data, offset);
            offset = this.ag.Deserialize(data, offset);
            offset = this.vels.Deserialize(data, offset);
            offset = this.shoulderImage.Deserialize(data, offset);
            offset = this.gripperImage.Deserialize(data, offset);
            
            var imq2ArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.imq2= new byte[imq2ArrayLength];
            for(var i = 0; i < imq2ArrayLength; i++)
            {
                this.imq2[i] = data[offset];
                offset += 1;
            }
            
            var imq3ArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.imq3= new byte[imq3ArrayLength];
            for(var i = 0; i < imq3ArrayLength; i++)
            {
                this.imq3[i] = data[offset];
                offset += 1;
            }
            
            var imq4ArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.imq4= new byte[imq4ArrayLength];
            for(var i = 0; i < imq4ArrayLength; i++)
            {
                this.imq4[i] = data[offset];
                offset += 1;
            }
            this.time = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MObservation: " +
            "\nproprio: " + proprio.ToString() +
            "\nag: " + ag.ToString() +
            "\nvels: " + vels.ToString() +
            "\nshoulderImage: " + shoulderImage.ToString() +
            "\ngripperImage: " + gripperImage.ToString() +
            "\nimq2: " + System.String.Join(", ", imq2.ToList()) +
            "\nimq3: " + System.String.Join(", ", imq3.ToList()) +
            "\nimq4: " + System.String.Join(", ", imq4.ToList()) +
            "\ntime: " + time.ToString();
        }
    }
}
