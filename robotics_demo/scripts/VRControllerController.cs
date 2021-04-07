using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.Rendering;
using static System.Linq.Enumerable;
using Unity.Robotics.ROSTCPConnector;
using JointPositions = RosMessageTypes.RoboticsDemo.JointPositions;
using PositionCommand = RosMessageTypes.RoboticsDemo.PositionCommand;
using QuaternionProprioState = RosMessageTypes.RoboticsDemo.QuaternionProprioState;
using Observation = RosMessageTypes.RoboticsDemo.Observation;
using ResetInfo = RosMessageTypes.RoboticsDemo.ResetInfo;
using AchievedGoal = RosMessageTypes.RoboticsDemo.AchievedGoal;
//
using RosMessageTypes.RoboticsDemo; // this is named after the folder you used to import msgs
using UnityEngine.XR; 
using OculusSampleFramework;

namespace RosSharp.Control
{
public class VRControllerController: MonoBehaviour
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

    public GameObject handAnchor;
    public bool useVR = true;
    public GameObject cameraTransform;
    public Camera shoulderCam;
    public Camera gripperCam;
    public RenderTexture shoulderRenderTexture;
    public RenderTexture gripperRenderTexture;
    private const int isBigEndian = 0;
    private const int step = 4;
    
    public GameObject obj1;

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
    public float publishMessageFrequency = 0.033f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    // Used so we can ignore messages stuck in the ros pipeline from before restarts
    private long restartTime;

    // The gripper angles are weird and do not correspond to the motor command, store requisite offsets
    // s.t open is 0 deg and closed is 40 deg here
    private float left_outer_knuckle_offset = -228.48f;
    // private float left_inner_finger_offset = -41.5f;
    // private float left_inner_knuckle_offset = 131.5f;
    // private float right_outer_knuckle_offset = -48.46f;
    // private float right_inner_finger_offset = -41.5f;
    // private float right_inner_knuckle_offset = -48.46f;

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

        //
        ROSConnection.instance.Subscribe<AchievedGoal>("reset_environment", resetEnvironment);


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
            print(joint);
        }
        EngageForceAfterInit();
        
        num_joints = articulationChain.Length;

        // Rendering stuff

        if (OVRManager.isHmdPresent & useVR) {
            print("VR Active");
        } else {
           // Make it so the VR rig just acts like any old camera
           Vector3 temp = new Vector3(-0.3f,-0.4f,-0.1f);
           cameraTransform.transform.position = temp;
           Vector3 rot = new Vector3(20f,30f,0f);
           cameraTransform.transform.rotation = Quaternion.Euler(rot);
        }

        shoulderRenderTexture = new RenderTexture(256, 256, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB); // this is very important
        shoulderRenderTexture.Create();
        gripperRenderTexture = new RenderTexture(64, 64, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB); // this is very important
        gripperRenderTexture.Create();

        reset_to_default();

        
        

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
                0.0F, // for the gripper
                unixTime // unix time so we can ignore commands which are sitting in ROS' pipes but from prev runs
            );
            // Finally send the message to server_endpoint.py running in ROS
        ros.Send("joint_commands", cmd);

        
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
            QuaternionState cmd = new QuaternionState(
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

    RosMessageTypes.Sensor.Image takeImage(RenderTexture renderTexture, Camera cam) {
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
        RosMessageTypes.Sensor.Image rosImage = new RosMessageTypes.Sensor.Image(new RosMessageTypes.Std.Header(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageBytes);
        cam.targetTexture = null;

        return rosImage;

    }

    AchievedGoal GetAchievedGoal() {

        Transform obj_pose = obj1.transform;

        AchievedGoal ag = new AchievedGoal(
                obj_pose.position.x, //start obj
                obj_pose.position.y,
                obj_pose.position.z,
                obj_pose.rotation.x,
                obj_pose.rotation.y,
                obj_pose.rotation.z,
                obj_pose.rotation.w); // end obj 

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
                (articulationChain[11].transform.localEulerAngles.z  + left_outer_knuckle_offset)/40.0F // gripper state from this
            );
            return proprio;
    }



    void publishObs()
    
        {
             // camera stuff
            RosMessageTypes.Sensor.Image gripperImage = takeImage(gripperRenderTexture, gripperCam);
            RosMessageTypes.Sensor.Image shoulderImage = takeImage(shoulderRenderTexture, shoulderCam);
            
            
            Observation state = new Observation(
                GetQuaternionProprioState(),
                GetAchievedGoal(), //getCurrentState(),
                shoulderImage,
                gripperImage);
            
            ros.Send("state", state);    
    }

    // void resetArticulation(int idx, int axis, float value) {
    //     //you cant just reset the ransform becasue then the x drive fights against it
    //     Vector3 originalRotation = articulationChain[idx].transform.eulerAngles;
    //     Vector3 resetRotation = new Vector3(originalRotation.x, originalRotation.y, originalRotation.z);
    //     resetRotation[axis] = value;
    //     //articulationChain[idx].transform.eulerAngles = resetRotation; // turns out doesn't work - some conflict between reset and set xdrive?
    //     set_articulation_position(idx, value);
    // }
    
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

        executeCommand(joints);

        resetEnvironment(GetAchievedGoal());
    }

    // Given a full state, resets
    void resetEnvironment(AchievedGoal ag) 
    {
        // reset object 1
        obj1.transform.rotation = new Quaternion(ag.obj1_q1, ag.obj1_q2, ag.obj1_q3, ag.obj1_q4);
        obj1.transform.position = new Vector3(ag.obj1_pos_x, ag.obj1_pos_y, ag.obj1_pos_z);

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

    
    // this subscriber is called when joint commands happen, which can be 
    // because a VR controller has pinged the IK server, or the joint sliders have sent out a topic
   void executeCommand(JointPositions command)
    {
        
        DateTime foo = DateTime.Now;
        long unixTime = ((DateTimeOffset)foo).ToUnixTimeSeconds();
        
        if (unixTime > restartTime+2) {
                //adding elements using collection-initializer syntax
                // command.shoulder = Math.Min(Math.Max(command.shoulder, -140F), 60F);
                // command.upper_arm = Math.Min(Math.Max(command.upper_arm, -80F), 90F);
                // command.forearm = Math.Min(Math.Max(command.forearm, -130F), -10F);
            var positions = new List<float>{ 
                0.00F, command.shoulder, command.upper_arm,command.forearm,command.wrist_1,command.wrist_2,command.wrist_3};
            // include the base non joint through the 0F
            foreach (int idx in Range(0,positions.Count)) {
                set_articulation_position(idx, positions[idx]);
            }
            grip(command.gripper);
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
            RPYState r = new RPYState(
                    proprio,
                    GetAchievedGoal() // randomize this up above if desired
            );
            ros.Send("reset", r);
        }
    }
    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            // print($"should {-articulationChain[1].transform.localEulerAngles.y}"  );
            // print($"upper {articulationChain[2].transform.localEulerAngles.x} " );
            // print($"forearm {articulationChain[3].transform.localEulerAngles.x}  ");
            // print($"w1 {articulationChain[4].transform.localEulerAngles.x} ");
            // print($"w2 {-articulationChain[5].transform.localEulerAngles.y}  ");

            print($" left_outer_knuckle {getKnucklePos(articulationChain[11].transform.localEulerAngles.z)} ");
            print($" left_inner_finger {getFingerPos(articulationChain[13].transform.localEulerAngles.z)} ");
            // print($" left_inner_knuckle {articulationChain[15].transform.localEulerAngles.z} ");
            // print($" right_outer_knuckle {articulationChain[16].transform.localEulerAngles.z} ");
            // print($" right_inner_finger {articulationChain[18].transform.localEulerAngles.z} ");
            // print($" right_inner_knuckle {articulationChain[20].transform.localEulerAngles.z} ");

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
                timeElapsed = 0;
        }   
    }
        

    void OnGUI()
    {
        int slider_y = 15;
        int slider_height = 10;
        if (joint_cntrl == true) {
            foreach (int idx in Range(0,jointSliderVals.Count)) {
                jointSliderVals[idx] = GUI.HorizontalSlider(new Rect(25, slider_y, 100, slider_height), jointSliderVals[idx], -180.0F, 180.0F);
                slider_y += slider_height;
            }
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
        
    }

    float getFingerPos(float f) {
        return -(f-41.5F)/40.0f;
    }

    float getKnucklePos(float k) {
        return (k  +  -228.48f)/40.0F; // gripper state from this
    }

}
}