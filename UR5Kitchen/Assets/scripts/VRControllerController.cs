using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.UI;
using static System.Linq.Enumerable;
using Unity.Robotics.ROSTCPConnector;
using JointPositions = RosMessageTypes.RoboticsDemo.MJointPositions;
using ResetAngles = RosMessageTypes.RoboticsDemo.MResetAngles;
using PositionCommand = RosMessageTypes.RoboticsDemo.MPositionCommand;
using QuaternionProprioState = RosMessageTypes.RoboticsDemo.MQuaternionProprioState;
using Observation = RosMessageTypes.RoboticsDemo.MObservation;
using ResetInfo = RosMessageTypes.RoboticsDemo.MResetInfo;
using Reengage = RosMessageTypes.RoboticsDemo.MReengage;
using AchievedGoal = RosMessageTypes.RoboticsDemo.MAchievedGoal;
using Velocities = RosMessageTypes.RoboticsDemo.MVelocities;
using getStateRequest = RosMessageTypes.RoboticsDemo.MgetStateRequest;
using getStateResponse = RosMessageTypes.RoboticsDemo.MgetStateResponse;
using getTimeRequest = RosMessageTypes.RoboticsDemo.MgetTimeRequest;
using getTimeResponse = RosMessageTypes.RoboticsDemo.MgetTimeResponse;
using ResetRequest = RosMessageTypes.RoboticsDemo.MresetRequest;
using ResetResponse = RosMessageTypes.RoboticsDemo.MresetResponse;
using RPYProprioState = RosMessageTypes.RoboticsDemo.MRPYProprioState;
using RPYState = RosMessageTypes.RoboticsDemo.MRPYState;
using Bool = RosMessageTypes.Std.MBool;
using ROString = RosMessageTypes.Std.MString;
using RosMessageTypes.RoboticsDemo; // this is named after the folder you used to import msgs
using UnityEngine.XR; 
using OculusSampleFramework;

namespace RosSharp.Control
{
public class VRControllerController: MonoBehaviour
{
    // Robot specific stuff
    private ArticulationBody[] articulationChain;
    private Collider[] colliders;
    public float stiffness = 100000;
    public float damping=100;
    public float forceLimit=1000;
    public float speed = 30f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 30f;// Units: m/s^2 / degree/s^2
    // End robot public stuff
    private int ee_index = 8; // this is the eelink

    public GameObject handAnchor;
    public bool useVR = true;
    public GameObject cameraTransform;
    public Camera shoulderCam;
    public Camera gripperCam;
    public RenderTexture shoulderRenderTexture;
    public RenderTexture gripperRenderTexture;
    private RosMessageTypes.Sensor.MImage shoulderImage;
    private RosMessageTypes.Sensor.MImage gripperImage;
    private const int isBigEndian = 0;
    private const int step = 4;
    
    public GameObject obj1;
    public GameObject obj2;
    public GameObject button1;
    private PhysicsButton button1_script;
    private GameObject button1_base;
 

    public GameObject button2;
    private PhysicsButton button2_script;
    private GameObject button2_base;

    public GameObject button3;
    private PhysicsButton button3_script;
    private GameObject button3_base;

    public GameObject drawer;
    private DrawerController drawer_script;

    public GameObject cupboard;
    private DoorController cupboard_script;

    private float x_min = -0.7f;
    private float x_max = -0.13f;
    private float z_min = 0.4f;
    private float z_max = 0.7f;
    private float y_button = 0.0569f;
    private float y_block = 0.18f;


    // Control / GUI specific stuff
    public bool joint_cntrl = false;
    public bool local_cntrl = false;
    // private List<float> jointSliderVals = new List<float>{ 
    //     0.00F, 
    //     -51.109F, 
    //     -102.771F,
    //     -109.448F,
    //     -147.852F,
    //     38.844F,
    //     0.00F,
    //     0.0F,
    //     0.0F,
    // };

    public Text recordingUI;
    private List<float> jointSliderVals = new List<float>{ 
        0.00F, 
        0.00F, 
        0.00F, 
        0.00F, 
        0.00F, 
        0.00F, 
        0.00F,
        0.0F,
        0.0F,
    };



    private List<float> XYZRPYSliderVals = new List<float>{ 
        -0.4F, 
        0.2F, 
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F
    };

    // this list contains the state field which we are accessing, then as keys it is which chain index it is 
    // and which dimension the value should set 
    // private List<(string info, int idx, int ax)> jointResetInfo = new List<(string info, int idx, int ax)> { 
    //         ("shoulder_link",  1, 1), // shoulder link, index 1, y axis
    //         ("upper_arm_link", 2, 0),
    //         ("forearm_link",   3, 0),
    //         ("wrist_1_link",   4, 0),
    //         ("wrist_2_link",   5, 1), 
    //         ("wrist_3_link",   6, 0),
    //         ("left_outer_knuckle", 11, 2),
    //         ("left_inner_finger",  13, 2),
    //         ("left_inner_knuckle", 15, 2),
    //         ("right_outer_knuckle", 16, 2),
    //         ("right_inner_finger",  18, 2),
    //         ("right_inner_knuckle", 20, 2),
    // };

    // private float threshold = 0.5F;

    private int num_joints;

    // Ros connection pub/subscribe stuff
    ROSConnection ros;

    // Publish the cube's position and rotation every N seconds
    public float period = 0.04f;
    private float nextPublishTime = 0.0f;

    // Used so we can ignore messages stuck in the ros pipeline from before restarts
    private long restartTime;

    // Reset 
    // 0. Start resetting flag
    // 1. Send joint commands, have arm converge on those positions 
    // 2. Send env reset positions, reset env to those positions
    // 3. Turn off the resetting flag
    private bool resetting = false;
    private ResetInfo resetState;

    private static readonly System.Random random = new System.Random();

    private int counter = 0;

    IEnumerator EngageForceAfterInit()
    {
        print("Start waiting");

        yield return new WaitForSeconds(2);
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            // Set up each of the joints
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            print(forceLimit);
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        
    }


    // Start is called before the first frame update
    void Start()
    {
        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevices(devices);

        foreach (var item in devices)
        {
            Debug.Log(item.name + item.characteristics);
        }
        // start the ROS connection
        ros = ROSConnection.instance;

        DateTime foo = DateTime.Now;
        restartTime = ((DateTimeOffset)foo).ToUnixTimeSeconds();

        // Set up a subscriber to listen to commands
        ROSConnection.instance.Subscribe<JointPositions>("joint_commands", executeCommand);

        // will reset to this state until de-engaged
        ROSConnection.instance.Subscribe<ResetInfo>("full_reset", fullReset);

        // release from 
        ROSConnection.instance.Subscribe<ROString>("saving_status", updateSavingStatus);
        ROSConnection.instance.Subscribe<Reengage>("re_engage_physics", stopResetting);
        ROSConnection.instance.Subscribe<Reengage>("re_engage_collision", reEngageCollision);
        ROSConnection.instance.Subscribe<Reengage>("toggleHalt", toggleHalt); // halt = True or False
        ROSConnection.instance.Subscribe<Reengage>("randomise", randomise); // halt = True or False


        ROSConnection.instance.ImplementService<getStateRequest>("getState", getStateServ);
        ROSConnection.instance.ImplementService<getTimeRequest>("getTime", getTimeServ);
        ROSConnection.instance.ImplementService<ResetRequest>("resetAndGetState", resetServ);


        
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
            currentDrive.forceLimit = 10;
            joint.xDrive = currentDrive;
        }
        StartCoroutine (EngageForceAfterInit());
        num_joints = articulationChain.Length;

        // Enivronment stuff
        button1_script = button1.GetComponent<PhysicsButton>();
        button2_script = button2.GetComponent<PhysicsButton>();
        button3_script = button3.GetComponent<PhysicsButton>();
        drawer_script = drawer.GetComponent<DrawerController>();
        cupboard_script = cupboard.GetComponent<DoorController>();

        // Rendering stuff

        if (OVRManager.isHmdPresent & useVR) {
            print("VR Active");
        } else {
           // Make it so the VR rig just acts like any old camera
           Vector3 temp = new Vector3(0.6f,-0.504f,0.48f);
           cameraTransform.transform.position = temp;
           Vector3 rot = new Vector3(51f,-82f,0.632f);
           cameraTransform.transform.rotation = Quaternion.Euler(rot);
        }

        shoulderRenderTexture = new RenderTexture(128,128, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB); // this is very important
        shoulderRenderTexture.Create();
        gripperRenderTexture = new RenderTexture(64, 64, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB); // this is very important
        gripperRenderTexture.Create();

        reset_to_default();

        colliders = this.gameObject.GetComponentsInChildren<Collider>();

        button1_base = button1.transform.parent.gameObject.transform.parent.gameObject;
        button2_base = button2.transform.parent.gameObject.transform.parent.gameObject;
        button3_base = button3.transform.parent.gameObject.transform.parent.gameObject;

        resetEnvironment(randomizeAchievedGoal());
    }
    
    // ####################################    Setters ######################################
    // This is to test when joint positions are broadcast
    void broadcastJointPositions() {
        DateTime foo = DateTime.Now;
        long unixTime = ((DateTimeOffset)foo).ToUnixTimeSeconds();

        JointPositions cmd = new JointPositions(
                jointSliderVals[1],// 1 - shoulder
                jointSliderVals[2],// 2 - upper arm
                jointSliderVals[3],// 3 - forearm 
                jointSliderVals[4],// 4 - wrist 1
                jointSliderVals[5],// 5 - wrist 2
                jointSliderVals[6],// 6 - wrist 3
                jointSliderVals[7],// for the gripper
                unixTime // unix time so we can ignore commands which are sitting in ROS' pipes but from prev runs
            );
            // Finally send the message to server_endpoint.py running in ROS
        ros.Send("joint_commands", cmd);


    }

    void setJointPositions(ResetAngles joints) {
        resetArticulation(1, 1, joints.shoulder);
        resetArticulation(2, 0, joints.upper_arm);
        resetArticulation(3, 0, joints.forearm);
        resetArticulation(4, 0, joints.wrist_1);
        resetArticulation(5, 1, joints.wrist_2);
        resetArticulation(6, 0, joints.wrist_3);
        resetArticulation(11, 2, joints.outer_knuckle_left);
        resetArticulation(13, 2, joints.inner_finger_left);
        resetArticulation(15, 2, joints.inner_knuckle_left);
        //resetArticulation(16, 2, joints.outer_knuckle_right);
        resetArticulation(18, 2, joints.inner_finger_right);
        resetArticulation(20, 2, joints.inner_knuckle_right);

        // special case
        Vector3 originalRotation = articulationChain[16].transform.eulerAngles;
        Vector3 resetRotation = new Vector3(-180,0, 0);
        resetRotation[2] = joints.outer_knuckle_right;
        articulationChain[16].transform.localEulerAngles = resetRotation; // turns out doesn't work - some conflict between reset and set xdrive?


        // special case
        originalRotation = articulationChain[20].transform.eulerAngles;
        resetRotation = new Vector3(-180,0, 0);
        resetRotation[2] = joints.inner_knuckle_right;
        articulationChain[20].transform.localEulerAngles = resetRotation; // turns out doesn't work - some conflict between reset and set xdrive?

    }


    void resetArticulation(int idx, int axis, float value) {
        //you cant just reset the ransform becasue then the x drive fights against it
        Vector3 originalRotation = articulationChain[idx].transform.eulerAngles;
        Vector3 resetRotation = new Vector3(0,0, 0);
        resetRotation[axis] = value;
        articulationChain[idx].transform.localEulerAngles = resetRotation; // turns out doesn't work - some conflict between reset and set xdrive?
        //set_articulation_position(idx, value);
    }

    // broadcast XYZRPY, which will get converted to joint commands by the ROS server - exactly as though they came from the AI!
    void commandXYZPositions() {
        PositionCommand cmd = new PositionCommand(
                XYZRPYSliderVals[0],// x
                XYZRPYSliderVals[1],// y
                XYZRPYSliderVals[2],// z 
                XYZRPYSliderVals[3],// r
                XYZRPYSliderVals[4],// p
                XYZRPYSliderVals[5],// y
                XYZRPYSliderVals[6]// gripper
            );
            // Finally send the message to server_endpoint.py running in ROS
        try
        {
            ros.Send("xyz_rpy_g_command", cmd);
        }
        catch 
        {
            print("Exception sending command");
        }
        

    }


    // broadcast XYZRPY, which will get converted to joint commands by the ROS server - exactly as though they came from the AI!
    void commandXYZQuatPositions() {

            Transform handPos = handAnchor.transform;
            // print();
            QuaternionProprioState cmd = new QuaternionProprioState(
                handPos.position.x,
                handPos.position.y,
                handPos.position.z,
                handPos.rotation.x,
                handPos.rotation.y,
                handPos.rotation.z,
                handPos.rotation.w,
                OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch)
            );
            // Finally send the message to server_endpoint.py running in ROS
        try
        {
            ros.Send("xyz_quat_g_command", cmd);
        }   
        catch 
        {
            print("Exception sending command");
        }
        

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

    RosMessageTypes.Sensor.MImage takeImage(RenderTexture renderTexture, Camera cam) {
        cam.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        cam.Render();
        Texture2D currentCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
        currentCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        currentCameraTexture.Apply();
        RenderTexture.active = currentRT;
        // Get the raw byte info from the screenshot
        byte[] imageBytes = currentCameraTexture.GetRawTextureData();
        uint imageHeight = (uint)renderTexture.height;
        uint imageWidth = (uint)renderTexture.width;
        RosMessageTypes.Sensor.MImage rosImage = new RosMessageTypes.Sensor.MImage(new RosMessageTypes.Std.MHeader(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageBytes);
        cam.targetTexture = null;
        renderTexture.DiscardContents();

        return rosImage;
        //return new RosMessageTypes.Sensor.Image();

    }

    static float RandomFloat(float min, float max){
        
        double val = (random.NextDouble() * (max - min) + min);
        return (float)val;
    }


    AchievedGoal GetAchievedGoal() {

        Transform obj_pose = obj1.transform;
        Transform button1pose = button1_base.transform;
        Transform button2pose = button2_base.transform;
        Transform button3pose = button3_base.transform;

        Vector3 obj2_position; 
        Quaternion obj2_rotation;
        Int16 obj2_present;

        if (obj2.activeInHierarchy) {
            obj2_rotation = obj2.transform.rotation;
            obj2_position = obj2.transform.position;
            obj2_present = 1;
            
        } else {
           obj2_rotation = new Quaternion(0f,0f,0f,0f);
           obj2_position = new Vector3(0, 0, 0);
           obj2_present = 0;
        }

        AchievedGoal ag = new AchievedGoal(
                obj_pose.position.x, //start obj
                obj_pose.position.y,
                obj_pose.position.z,
                obj_pose.rotation.x,
                obj_pose.rotation.y,
                obj_pose.rotation.z,
                obj_pose.rotation.w,  // end obj 
                button1_script.value,
                button2_script.value,
                button3_script.value,
                drawer_script.value,
                cupboard_script.value,
                obj2_present,
                obj2_position.x, //start obj
                obj2_position.y,
                obj2_position.z,
                obj2_rotation.x,
                obj2_rotation.y,
                obj2_rotation.z,
                obj2_rotation.w,  // end obj 
                button1pose.position.x,
                button1pose.position.y,
                button1pose.position.z,
                button2pose.position.x,
                button2pose.position.y,
                button2pose.position.z,
                button3pose.position.x,
                button3pose.position.y,
                button3pose.position.z);

        return ag;

    }

    QuaternionProprioState GetQuaternionProprioState() {
        // arm information
            Transform grasp_target = articulationChain[ee_index].transform;
            Transform ee_info = articulationChain[ee_index-1].transform;
        // 
            QuaternionProprioState proprio = new QuaternionProprioState(
                grasp_target.position.x, //start arm
                grasp_target.position.y,
                grasp_target.position.z,
                ee_info.rotation.x,
                ee_info.rotation.y,
                ee_info.rotation.z,
                ee_info.rotation.w,
                getKnucklePos(articulationChain[11].transform.localEulerAngles.z) // gripper state from this
            );
            return proprio;
    }

    
    Velocities GetVelocities() {
        Rigidbody rb = obj1.GetComponent<Rigidbody>();
        Velocities v = new Velocities(
            rb.velocity[0],
            rb.velocity[1],
            rb.velocity[2],
            rb.angularVelocity[0],
            rb.angularVelocity[1], 
            rb.angularVelocity[2]
            );
            return v;
    }


    Observation getState() {
                     // camera stuff
            shoulderImage = takeImage(shoulderRenderTexture, shoulderCam);
            gripperImage = takeImage(gripperRenderTexture, gripperCam);

            int imageQuadrant = shoulderImage.data.Length / 4;

            byte[] q1 = shoulderImage.data.Take(imageQuadrant).ToArray();
            byte[] q2 = shoulderImage.data.Skip(imageQuadrant).Take(imageQuadrant).ToArray();
            byte[] q3 = shoulderImage.data.Skip(imageQuadrant*2).Take(imageQuadrant).ToArray();
            byte[] q4 = shoulderImage.data.Skip(imageQuadrant*3).ToArray();

            shoulderImage.data = q1;

            return new Observation(
            GetQuaternionProprioState(),
            GetAchievedGoal(), //getCurrentState(),
            GetVelocities(),
            shoulderImage,
            gripperImage,
            q2,
            q3,
            q4,
            ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds(),
            GetJointPositions());

    }
    void publishObs() {
        ros.Send("state", getState());    
    }

    getStateResponse getStateServ(getStateRequest req) {
        getStateResponse resp = new getStateResponse();
        resp.state = getState();
        return resp;

    }


    getTimeResponse getTimeServ(getTimeRequest req) {
        getTimeResponse resp = new getTimeResponse();
        resp.time_rec = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();
        return resp;
    }



    
    
    // bool convergingToReset(ResetInfo state) {

    //     // Would be great to so something like this, but get properties doesnt seem to work 
    //     // PropertyInfo[] properties = state.GetProperties();
    //     //     print(properties);
    //     //     foreach (PropertyInfo pi in properties)
    //     //     {
    //     //         print(string.Format("Name: {0} | Value: {1}", pi.Name, pi.GetValue(proprio, null)));
    //     //     }
        

    //     if ((Math.Abs(-articulationChain[1].transform.eulerAngles.y - state.shoulder_link) < threshold)&
    //     (Math.Abs(wrapEuler(articulationChain[2].transform.eulerAngles.x) -  state.upper_arm_link) < threshold)&
    //     (Math.Abs(wrapEuler(articulationChain[3].transform.eulerAngles.x) -  state.forearm_link) < threshold)&
    //     (Math.Abs(wrapEuler(articulationChain[4].transform.eulerAngles.x) -  state.wrist_1_link) < threshold)&
    //     (Math.Abs(wrapEuler(-articulationChain[5].transform.eulerAngles.y) -  state.wrist_2_link) < threshold)&
    //     (Math.Abs(wrapEuler(articulationChain[6].transform.eulerAngles.x) -  state.wrist_3_link) < threshold) &
    //     // (Math.Abs(articulationChain[11].transform.eulerAngles.z + left_outer_knuckle_offset - state.left_outer_knuckle) < threshold)&
    //     // (Math.Abs(articulationChain[13].transform.eulerAngles.z + left_inner_finger_offset -  state.left_inner_finger) < threshold)&
    //     // (Math.Abs(articulationChain[15].transform.eulerAngles.z + left_inner_knuckle_offset -  state.left_inner_knuckle) < threshold)&
    //     // (Math.Abs(articulationChain[16].transform.eulerAngles.z + right_outer_knuckle_offset -  state.right_outer_knuckle) < threshold)&
    //     // (Math.Abs(articulationChain[18].transform.eulerAngles.z + right_inner_finger_offset -  state.right_inner_finger) < threshold)&
    //     // (Math.Abs(articulationChain[20].transform.eulerAngles.z + right_inner_knuckle_offset -  state.right_inner_knuckle) < threshold)&
    //     (Math.Abs(obj1.transform.position.x -  state.obj1_pos_x) < threshold) &
    //     (Math.Abs(obj1.transform.position.y -  state.obj1_pos_y) < threshold) &
    //     (Math.Abs(obj1.transform.position.z -  state.obj1_pos_z) < threshold) &
    //     (Math.Abs(obj1.transform.rotation.x -  state.obj1_q1) < threshold) &
    //     (Math.Abs(obj1.transform.rotation.y -  state.obj1_q2) < threshold) &
    //     (Math.Abs(obj1.transform.rotation.z -  state.obj1_q3) < threshold) &
    //     (Math.Abs(obj1.transform.rotation.w -  state.obj1_q4) < threshold))
    //     {

            
    //         return false;
    //     } else {
    //         //print(wrapEuler(articulationChain[2].transform.eulerAngles.x));
            
    //         print(state.forearm_link);
    //         print(   (Math.Abs(wrapEuler(articulationChain[3].transform.eulerAngles.x) -  state.forearm_link) < threshold)   );
    //         // print(state.left_outer_knuckle);
    //         // print(articulationChain[11].transform.rotation.z);
    //         // print(articulationChain[11].transform.eulerAngles.z + left_outer_knuckle_offset);
    //         // print(((Math.Abs(articulationChain[11].transform.eulerAngles.z + left_outer_knuckle_offset - state.left_outer_knuckle) < threshold)));
    //         return false;
    //     }
    // }


    void reset_to_default() {


        JointPositions joints = new JointPositions();
        joints.shoulder = -51.0F;
        joints.upper_arm = -12.7F;
        joints.forearm = -109.44F;
        joints.wrist_1 = -57.85F;
        joints.wrist_2 = 38.8F;
        joints.wrist_3 = 0.0F;
        joints.gripper = 0.0F;

        articulate_joints(joints);
    }

        AchievedGoal randomizeAchievedGoal() {


            AchievedGoal ag = new AchievedGoal(
                    RandomFloat(x_min, x_max), //start obj
                    y_block,
                    RandomFloat(z_min, z_max),
                    0f,
                    0f,
                    0f,
                    1f,  // end obj 
                    0f,  //button 1
                    0f,  //button 2
                    0f, //button 3
                    0f, // drawer
                    0f, // door
                    1, // obj2 present
                    RandomFloat(x_min, x_max), //start obj
                    y_block,
                    RandomFloat(z_min, z_max),
                    0f,
                    0f,
                    0f,
                    1f,  // end obj  
                    RandomFloat(x_min, x_max), //start obj
                    y_button,
                    RandomFloat(z_min, z_max),
                    RandomFloat(x_min, x_max), //start obj
                    y_button,
                    RandomFloat(z_min, z_max),
                    RandomFloat(x_min, x_max), //start obj
                    y_button,
                    RandomFloat(z_min, z_max));

            return ag;
    }

    // Given a full state, resets - this resets the simple state information
    void resetEnvironment(AchievedGoal ag) 
    {
        // reset object 1
        obj1.transform.rotation = new Quaternion(ag.obj1_q1, ag.obj1_q2, ag.obj1_q3, ag.obj1_q4);
        obj1.transform.position = new Vector3(ag.obj1_pos_x, ag.obj1_pos_y, ag.obj1_pos_z);
        drawer_script.Reset(ag.drawer);
        cupboard_script.Reset(ag.door);
        button1_script.Reset(ag.button1);
        button2_script.Reset(ag.button2);
        button3_script.Reset(ag.button3);

        // var vec = new Vector2(ag.obj2_pos_x, ag.obj1_pos_x);
        if (ag.obj2_present == 1) {
            obj2.transform.rotation = new Quaternion(ag.obj2_q1, ag.obj2_q2, ag.obj2_q3, ag.obj2_q4);
            obj2.transform.position = new Vector3(ag.obj2_pos_x, ag.obj2_pos_y, ag.obj2_pos_z);
            button1_base.transform.position = new Vector3(ag.button1_x, ag.button1_y, ag.button1_z);
            button2_base.transform.position  = new Vector3(ag.button2_x, ag.button2_y, ag.button2_z);
            button3_base.transform.position  = new Vector3(ag.button3_x, ag.button3_y, ag.button3_z);
        }

    }
    
    void fullReset(ResetInfo r) {
        // turn off collision so we can effectively reset
        foreach (Collider c in colliders) {
            c.enabled = false;
        }
        
        // by setting this true and that to the update state, every update step will force this changes
        resetState = r;
        resetting = true;
    }

    // when we want the sim back on - collisoins, not resetting. Will be de_engged the entire time we are capturing photos post fact.
    void stopResetting(Reengage r) {
        // get out of resetting mode
        resetting = false;

        foreach (Collider c in colliders) {
            c.enabled = false;
        }

        Time.timeScale = 1f;

    }

    void reEngageCollision(Reengage r) {
        // turn collision back on, this is separated from the physics for photo taking purposes
        foreach (Collider c in colliders) {
            c.enabled = true;
        }
    }

    void toggleHalt(Reengage r) {
        if (r.i == 0) {
            Time.timeScale = 0f;
        } else {
            Time.timeScale = 1f;
        }
    }


   // ####################### Subscribers ##############################################
   void grip(float percent) {
        // Left hand side of the gripper
        
        float outer_knuckle_closed = 40;
        float inner_finger_closed = 40;
        float inner_knuckle_closed = -40;

        // the outer knuckle should be constrainted to how closed the inner finger is, and the inner knuckle to how closed the outer knuckle is
        // inner fingers
        // outer knuckles
        set_articulation_position(11, percent*outer_knuckle_closed);
        set_articulation_position(16, percent*-outer_knuckle_closed);

        set_articulation_position(13, Math.Min(percent, getKnucklePos(articulationChain[11].transform.localEulerAngles.z))*inner_finger_closed);
        set_articulation_position(18, Math.Min(percent, getKnucklePos(articulationChain[16].transform.localEulerAngles.z))*inner_finger_closed);
        
        
        //inner knuckles
        set_articulation_position(15, Math.Min(percent, getKnucklePos(articulationChain[11].transform.localEulerAngles.z))*inner_knuckle_closed);
        set_articulation_position(20, Math.Min(percent, getKnucklePos(articulationChain[16].transform.localEulerAngles.z))*inner_knuckle_closed);


    }

    void set_articulation_position(int idx, float position)
    {
        ArticulationDrive currentDrive = articulationChain[idx].xDrive;
        currentDrive.target = position;
        articulationChain[idx].xDrive = currentDrive;
    }

    void articulate_joints(JointPositions command) {

            var positions = new List<float>{ 
                0.00F, command.shoulder, command.upper_arm,command.forearm,command.wrist_1,command.wrist_2,command.wrist_3};
            // include the base non joint through the 0F
            foreach (int idx in Range(0,positions.Count)) {
                set_articulation_position(idx, positions[idx]);
            }
            grip(command.gripper);

    }
    // this subscriber is called when joint commands happen, which can be 
    // because a VR controller has pinged the IK server, or the joint sliders have sent out a topic
   void executeCommand(JointPositions command)
    {

        long unixTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeSeconds();
        
        if (unixTime > restartTime+2) {
                //adding elements using collection-initializer syntax
                // command.shoulder = Math.Min(Math.Max(command.shoulder, -140F), 60F);
                // command.upper_arm = Math.Min(Math.Max(command.upper_arm, -80F), 90F);
                // command.forearm = Math.Min(Math.Max(command.forearm, -130F), -10F);
                
                
            articulate_joints(command);
        }
        
    }

    void checkButtons() {

        if (OVRInput.GetDown(OVRInput.RawButton.X)) {
            print("resetting");
            // Premeptively reset it to a nice position
            JointPositions joints = new JointPositions();
            joints.shoulder = -51.0F;
            joints.upper_arm = -12.7F;
            joints.forearm = -109.44F;
            joints.wrist_1 = -57.85F;
            joints.wrist_2 = 38.8F;
            joints.wrist_3 = 0.0F;
            joints.gripper = 0.0F;

            executeCommand(joints);

            // Block the arm from accepting any new joint commands for 2s
            DateTime foo = DateTime.Now;
            restartTime = ((DateTimeOffset)foo).ToUnixTimeSeconds();

            // create an actual reset request to the big controller in the sky
            // s.t it doesn't send any new requests or save any data etc, saves the trajectory etc
            RPYProprioState proprio = new RPYProprioState();
            proprio.pos_x = -0.4F;
            proprio.pos_y = 0.2F;
            proprio.pos_z = 0.0F;
            resetEnvironment(randomizeAchievedGoal());
            RPYState r = new RPYState(
                    proprio,
                    GetAchievedGoal() // randomize this up above if desired
            );
            ros.Send("reset", r);
            
        }
        if (OVRInput.GetDown(OVRInput.RawButton.Y)) {
            Bool b = new Bool();
            ros.Send("start_recording",b);
            
            
        }

    }

    void updateSavingStatus(ROString str) {
        string disp_str = str.ToString().Split(':')[2];
        recordingUI.text =disp_str;
        if (String.Equals(disp_str, "Recording")) {
            recordingUI.color = Color.green;
        } else {
            recordingUI.color = Color.blue;
        }

    }
    // Update is called once per frame
    void Update()
    {

        if (resetting) {
            // constantly reset state if we are resetting to ensure maximal alignment (e.g objects will hang in air)
            resetEnvironment(resetState.ag);
            articulate_joints(resetState.joints); // this needs to be sent continuosly because the gripper constraint (that interior bits can't go beyond the angle of exterior parts means it converges over multiple steps)
        } 
        else if (Time.time > nextPublishTime)
        {

            nextPublishTime += period;
            

            // if (resettingState) { // if we are in the middle of resetting state, check if it has converged
            //     resetState(resetSt); // recall this to reset transforms if theyve been moved as the articulation moves into posiitoni
            //     resettingState = convergingToReset(resetSt);
            //     // print("Still Resetting");

            // }  else { 
            if (joint_cntrl & local_cntrl) {// commanding from here not from some AI
                broadcastJointPositions();
                }
            else if (local_cntrl) { // commanding from here not from some AI
                commandXYZPositions();
                }
            else if (OVRManager.isHmdPresent & useVR & !local_cntrl) {
                checkButtons();
                commandXYZQuatPositions();
                }
                    
            publishObs();
                
        }   
    }
        

    void OnGUI()
    {
        //Time.timeScale = 0f;        
        int slider_y = 15;
        int slider_height = 10;
        if (joint_cntrl == true) {
            foreach (int idx in Range(0,7)) {
                jointSliderVals[idx] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), jointSliderVals[idx], -180.0F, 180.0F);
                slider_y += slider_height;
            }
            jointSliderVals[7] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), jointSliderVals[7], -1F, 1F);
        }
        else 
        {
            foreach (int idx in Range(0,XYZRPYSliderVals.Count-1)) {
                XYZRPYSliderVals[idx] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), XYZRPYSliderVals[idx],-1.5F, 1.5F);
                slider_y += slider_height;



            }
            // Different range on the gripper
            XYZRPYSliderVals[XYZRPYSliderVals.Count-1] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), XYZRPYSliderVals[XYZRPYSliderVals.Count-1],-0F, 1F);
        }

        // ResetAngles ra = ResetAngles(
        //     jointSliderVals[1],
        //     jointSliderVals[2],
        //     jointSliderVals[3],
        //     jointSliderVals[4],
        //     jointSliderVals[5],
        //     jointSliderVals[6],
        // ) 
        
        //  setJointPositions(new ResetAngles());
        // broadcastJointPositions();
         //GetJointPositions();
         
        
    }

    float getFingerPos(float f) {
        return -(f-41.5F)/40.0f;
    }

    float getKnucklePos(float k) {
        return (k-228.48f)/40.0F; // gripper state from this
    }

    // Local angles doesn't return the same angle as is set by set transform (that aligns with the inspector value)
    // Additionally, there is an issue where +/-10 (for example) of 90 degress is 80 degrees local angles both directions! 
    // You can tell which way it is, because every second local angles z is 180 when it is on the wrong side. \
    // Absurd? Yes.  
    float lastUpperArmIndicator = 0;
    float lastForeArmIndicator = 0;
    float lastw1Indicator = 0;
    float lastw2Indicator = 0;
    float lastw3Indicator = 0;


    float disambiguate_joint_x(Vector3 angle, float indicator) {
        float a ;
        if (angle.x < 180.0F) {
            a = angle.x;

            if ((indicator > 1) || (angle.z > 1)) {
                print($" {indicator} {angle.z}");
                a = 180-angle.x;
            }
        } else {
            a = angle.x - 360.0F;
            
            if ((indicator > 1) || (angle.z > 1)) {
                a = ( 270 - (angle.x-270)) - 360.0F;
            }
            //print($"upper {a} {angle} {indicator}");
        }
        return a;
    }

    float disambiguate_joint_y(Vector3 angle) {
        float a;
        if (angle.y > 180.0F) {
            a = angle.y - 360.0f;
        } else {
            a = angle.y;
        }
        return a;
    }

    ResetResponse resetServ(ResetRequest r) {
        foreach (Collider c in colliders) {
            c.enabled = false;
        }
        Time.timeScale = 0f;  

        setJointPositions(r.resetAngles);
        resetEnvironment(r.ag);
        ResetResponse resp = new ResetResponse();
        resp.state = getState();
        return resp;

    }

    ResetAngles GetJointPositions() {
        float shoulder_ang, upper_ang, forearm_ang, w1_ang, w2_ang, w3_ang;


        Vector3 shoulder = articulationChain[1].transform.localEulerAngles;
        
        // print($"shoulder {shoulder_ang}");
        // Vector3 upper = articulationChain[2].transform.localEulerAngles;
        shoulder_ang  = disambiguate_joint_y(articulationChain[1].transform.localEulerAngles);
        upper_ang = disambiguate_joint_x(articulationChain[2].transform.localEulerAngles, lastUpperArmIndicator);
        forearm_ang = disambiguate_joint_x(articulationChain[3].transform.localEulerAngles, lastForeArmIndicator);
        w1_ang = disambiguate_joint_x(articulationChain[4].transform.localEulerAngles, lastw1Indicator);
        w2_ang = disambiguate_joint_y(articulationChain[5].transform.localEulerAngles);
        w3_ang =  disambiguate_joint_x(articulationChain[6].transform.localEulerAngles, lastw3Indicator);
        // print($"check {counter} {w1_ang} {articulationChain[4].transform.localEulerAngles} {lastw1Indicator}");
        // print($"upper {counter} {w3_ang} {articulationChain[6].transform.localEulerAngles}");//.x}, {articulationChain[2].transform.localEulerAngles.y} " );
        //
       //.x}, {articulationChain[2].transform.localEulerAngles.y} " );
        // print($"check {counter} {w2_ang} {articulationChain[5].transform.localEulerAngles}");
        // print($"forearm {articulationChain[3].transform.localEulerAngles.x}  ");
        // print($"w1 {articulationChain[4].transform.localEulerAngles.x} ");
        // print($"w2 {articulationChain[5].transform.localEulerAngles.y}  ");

        // print($" left_outer_knuckle {getKnucklePos(articulationChain[11].transform.localEulerAngles.z)} ");
        // print($" left_inner_finger {getFingerPos(articulationChain[13].transform.localEulerAngles.z)} ");
        // print($" left_inner_knuckle {articulationChain[15].transform.localEulerAngles.z} ");
        // print($" right_outer_knuckle {articulationChain[16].transform.localEulerAngles.z} ");
        // print($" right_inner_finger {articulationChain[18].transform.localEulerAngles.z} ");
        // print($" right_inner_knuckle {articulationChain[20].transform.localEulerAngles.z} ");

        float lao = articulationChain[11].transform.localEulerAngles.z;
        float lif = articulationChain[13].transform.localEulerAngles.z;
        float lik = articulationChain[15].transform.localEulerAngles.z;
        float rao = articulationChain[16].transform.localEulerAngles.z-180f;
        float rif = articulationChain[18].transform.localEulerAngles.z;
        float rik = articulationChain[20].transform.localEulerAngles.z-180f;

        lastUpperArmIndicator = articulationChain[2].transform.localEulerAngles.z;
        lastForeArmIndicator = articulationChain[3].transform.localEulerAngles.z;
        lastw1Indicator = articulationChain[4].transform.localEulerAngles.z;
        lastw2Indicator = articulationChain[5].transform.localEulerAngles.z;
        lastw3Indicator = articulationChain[6].transform.localEulerAngles.z;
        counter += 1;
        return new ResetAngles(shoulder_ang, upper_ang, forearm_ang, w1_ang, w2_ang, w3_ang, lao, lif, lik, rao, rif, rik);

    }

    void randomise(Reengage i) {
        print("Randomisg");
        GameObject light = GameObject.Find("Directional Light");
        Vector3 new_ori = new Vector3(RandomFloat(20, 80), RandomFloat(-180, 180) ,RandomFloat(-5, 5));
        light.transform.eulerAngles = new_ori;

        GameObject table = GameObject.Find("PRE_FUR_Kitchen_counter_01_07");
        Material[] mats = table.GetComponent<Renderer>().materials;
        mats[3].color =  UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
        mats[1].color =  UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
        table.GetComponent<Renderer>().materials = mats;

        GameObject door = GameObject.Find("Door");
        door.GetComponent<Renderer>().material.color =  UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);



    }







}}

// Our rules
// shoulder - y - if > 0, y. if >180, 360 - y
// 