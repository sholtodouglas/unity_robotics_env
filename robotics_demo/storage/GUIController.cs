using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.Rendering;
using static System.Linq.Enumerable;
using Unity.Robotics.ROSTCPConnector;

//
using RosMessageTypes.RoboticsDemo; // this is named after the folder you used to import msgs

namespace RosSharp.Control
{
public class GUIController : MonoBehaviour
{
    // Robot specific stuff
    private ArticulationBody[] articulationChain;
    public float stiffness = 10000;
    public float damping=100;
    public float forceLimit=1000;
    public float speed = 30f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 10f;// Units: m/s^2 / degree/s^2
    // End robot public stuff
    private int ee_index = 8; // this is the eelink - TODO: adapted once we have a gripper

    // Control / GUI specific stuff
    public bool joint_cntrl = true;
    private List<float> sliderVals = new List<float>{ 
        0.00F, 
        -51.109F, 
        -102.771F,
        -109.448F,
        -147.852F,
        38.844F,
        0.00F,
        0.0F,
        0.0F,
    };
    private int num_joints;

    // Ros connection pub/subscribe stuff
    ROSConnection ros;
    public string topicName = "pos_rot";

    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    // Start is called before the first frame update
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.instance;

        // Set up the arm variables
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>(); // https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody.html?_ga=2.54684075.1087433992.1613790814-228562203.1613145667
        int defDyanmicVal = 10;
        
        foreach (ArticulationBody joint in articulationChain)
        {
            // Set up each of the joints
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
            print(joint);
        }
        
        num_joints = articulationChain.Length;
        
    }
    
    // ####################################    Setters ######################################
    // TODO: Gripper - this should be a subscriber which just sets joint positions whenever it gets a joint command msg
    void commandJointPositions(List<float> positions) {
        // Update the joint positions from a float array.
        // 0 - base (controls nothing)
        // 1 - shoulder
        // 2 - upper arm 
        // 3 - forearm 
        // 4 - wrist 1 
        // 5 - wrist 2
        // 6 - wrist 3
        // 7 - ee link

        foreach (int idx in Range(0,positions.Count)) {
            ArticulationDrive currentDrive = articulationChain[idx].xDrive;
            currentDrive.target = positions[idx];
            articulationChain[idx].xDrive = currentDrive;
        }
    }

    void commandXYZPositions(List<float> positions) {

    }

    // #################################  Getters ############################################

    // void getPosOri(ArticulationBody link)
    // {
    //     Transform data = link.transform;
    //     // //print(data.eulerAngles);
    //     // print(data.position);
    //     return 
    // }

    // void getEEinfo()
    // {
        
    // }


    // ################################# Publishers #################################

    void publishStates()
        {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            Transform grasp_target = articulationChain[ee_index].transform;
            Transform ee_info = articulationChain[ee_index-1].transform;
            PosRot eePosRot = new PosRot(
                grasp_target.position.x,
                grasp_target.position.y,
                grasp_target.position.z,
                ee_info.rotation.x,
                ee_info.rotation.y,
                ee_info.rotation.z,
                ee_info.rotation.w
            );
            print(ee_info.rotation);
            print(ee_info.eulerAngles);
            print(grasp_target.position);
            // Finally send the message to server_endpoint.py running in ROS
            ros.Send(topicName, eePosRot);

            timeElapsed = 0;
        }
    }



    // Update is called once per frame
    void Update()
    {
        
        if (joint_cntrl == true) 
        {
            commandJointPositions(sliderVals);
        }
        else 
        {
            commandXYZPositions(sliderVals);
        }
        publishStates();
    }

    void OnGUI()
    {
        int slider_y = 25;
        int slider_height = 30;
        foreach (int idx in Range(0,num_joints)) {
            sliderVals[idx] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), sliderVals[idx], -180.0F, 180.0F);
            slider_y += slider_height;
        }
        
    }
    
}
}