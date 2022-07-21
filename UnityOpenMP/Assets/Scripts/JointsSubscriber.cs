using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using JointStates_Msg = RosMessageTypes.Sensor.JointStateMsg;


public class JointsSubscriber : MonoBehaviour
{   
    [SerializeField]
    GameObject m_OpenMP;
    [SerializeField]
    string Topic = "/joint_states";

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;
    
    
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const int k_IntupJoints = 8;
    const float pi = 3.1416f;
    float[] joint_position = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};




    private void Start()
    {   
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_OpenMP.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/gripper_base/right_gripper";
        var leftGripper = linkName + "/gripper_base/left_gripper";

        m_RightGripper = m_OpenMP.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_OpenMP.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

 
    private void SetJoint()
    {
        // Set the joint values for every joint
        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                joint1XDrive.target = joint_position[joint];
                m_JointArticulationBodies[joint].xDrive = joint1XDrive;
            }
        
        // Set the gripper
        // Hardcoded variables
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = joint_position[7];
        rightDrive.target = joint_position[6];

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    
    }

    public void StartSetJoint()
    {
        // Get ROS connection static instance
        ROSConnection.GetOrCreateInstance().Subscribe<JointStates_Msg>(Topic, ReceiveMessage);

    }


    private void ReceiveMessage(JointStates_Msg message)
    {
        
        for (var i = 0; i < k_IntupJoints; i++)
        {
            joint_position[i] = (float)message.position[i]*180/pi;
        }

        SetJoint();

                
    }

}
