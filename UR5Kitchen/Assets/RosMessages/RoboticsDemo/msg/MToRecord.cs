//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MToRecord : Message
    {
        public const string RosMessageName = "robotics_demo/ToRecord";

        public MRPYState state;
        public MVelocities vels;
        public Sensor.MImage shoulderImage;
        public Sensor.MImage gripperImage;
        public MPositionCommand a;
        public int timestep;
        public double data_gen_time;
        public double data_arrival_time;
        public double data_processed_time;
        public double beat_sent_time;
        public double act_begin_time;
        public double model_processed_time;

        public MToRecord()
        {
            this.state = new MRPYState();
            this.vels = new MVelocities();
            this.shoulderImage = new Sensor.MImage();
            this.gripperImage = new Sensor.MImage();
            this.a = new MPositionCommand();
            this.timestep = 0;
            this.data_gen_time = 0.0;
            this.data_arrival_time = 0.0;
            this.data_processed_time = 0.0;
            this.beat_sent_time = 0.0;
            this.act_begin_time = 0.0;
            this.model_processed_time = 0.0;
        }

        public MToRecord(MRPYState state, MVelocities vels, Sensor.MImage shoulderImage, Sensor.MImage gripperImage, MPositionCommand a, int timestep, double data_gen_time, double data_arrival_time, double data_processed_time, double beat_sent_time, double act_begin_time, double model_processed_time)
        {
            this.state = state;
            this.vels = vels;
            this.shoulderImage = shoulderImage;
            this.gripperImage = gripperImage;
            this.a = a;
            this.timestep = timestep;
            this.data_gen_time = data_gen_time;
            this.data_arrival_time = data_arrival_time;
            this.data_processed_time = data_processed_time;
            this.beat_sent_time = beat_sent_time;
            this.act_begin_time = act_begin_time;
            this.model_processed_time = model_processed_time;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(state.SerializationStatements());
            listOfSerializations.AddRange(vels.SerializationStatements());
            listOfSerializations.AddRange(shoulderImage.SerializationStatements());
            listOfSerializations.AddRange(gripperImage.SerializationStatements());
            listOfSerializations.AddRange(a.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.timestep));
            listOfSerializations.Add(BitConverter.GetBytes(this.data_gen_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.data_arrival_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.data_processed_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.beat_sent_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.act_begin_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.model_processed_time));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.state.Deserialize(data, offset);
            offset = this.vels.Deserialize(data, offset);
            offset = this.shoulderImage.Deserialize(data, offset);
            offset = this.gripperImage.Deserialize(data, offset);
            offset = this.a.Deserialize(data, offset);
            this.timestep = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.data_gen_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.data_arrival_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.data_processed_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.beat_sent_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.act_begin_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.model_processed_time = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MToRecord: " +
            "\nstate: " + state.ToString() +
            "\nvels: " + vels.ToString() +
            "\nshoulderImage: " + shoulderImage.ToString() +
            "\ngripperImage: " + gripperImage.ToString() +
            "\na: " + a.ToString() +
            "\ntimestep: " + timestep.ToString() +
            "\ndata_gen_time: " + data_gen_time.ToString() +
            "\ndata_arrival_time: " + data_arrival_time.ToString() +
            "\ndata_processed_time: " + data_processed_time.ToString() +
            "\nbeat_sent_time: " + beat_sent_time.ToString() +
            "\nact_begin_time: " + act_begin_time.ToString() +
            "\nmodel_processed_time: " + model_processed_time.ToString();
        }
    }
}
