namespace EE.TalTech.IVAR.Robotics.MoveItIntegration
{
    using System;
    using System.Linq;
    using Robotics.ROSIndustrial;
    using RosMessageTypes.Moveit;
    using Unity.Robotics.ROSTCPConnector.ROSGeometry;
    using UnityEngine;

    /// <summary>
    /// Provides functionality to compute connected robot's IK through MoveIt IK service running in ROS. 
    /// </summary>
    public class MoveItIKService : RosConnectedBehaviour
    {
        #region Variables

        /// <summary>
        /// ROS topic of the IK service.
        /// </summary>
        public string serviceTopic = "compute_ik";
        
        /// <summary>
        /// Robot kinematics data.
        /// </summary>
        public UrdfRobotKinematicsDataProvider robotKinematics;

        /// <summary>
        /// Maximum amount of time to allow for kinematics solver to produce a solution.
        /// Computation is aborted if this time limit is reached.
        /// </summary>
        /// <remarks>
        /// This does not account for network latency between Unity and ROS, so the actual time of aborted request may be longer.
        /// </remarks>
        public float solutionTimeout = 0.1f;

        #endregion

        #region Unity Callbacks

        private void OnEnable()
        {
            bool isConnected = !rosConnection.HasConnectionError && rosConnection.HasConnectionThread;
                
            rosConnection.RegisterRosService<GetPositionIKRequest, GetPositionIKResponse>(serviceTopic);
        }

        #endregion

        #region Public Methods
        
        /// <summary>
        /// Computes positions of robot's joints required to reach given pose.
        /// </summary>
        /// <param name="targetWorldPose">Target IK pose in Unity's World space.</param>
        /// <param name="resultsHandler">Callback to process the IK results.</param>
        public void ComputeIK(Pose targetWorldPose, Action<GetPositionIKResponse> resultsHandler)
        {
            if (!isActiveAndEnabled)
            {
                Debug.LogWarning($"IK solution will not be computed because this {nameof(MoveItIKService)} component is disabled.", this);
                return;
            }
        
            // Convert target pose to robot coordinate system
            var ikPose = new Pose
            {
                position = robotKinematics.RootLink.transform.InverseTransformPoint(targetWorldPose.position),
                rotation = Quaternion.Inverse(robotKinematics.RootLink.transform.rotation) * targetWorldPose.rotation
            };

            // Craft service request
            var ikServiceRequest = new GetPositionIKRequest
            {
                ik_request =
                {
                    avoid_collisions = true,
                    group_name = "arm",
                    timeout =
                    {
                        sec = Mathf.FloorToInt(solutionTimeout),
                        nanosec = (int) (solutionTimeout * 1000000000 % 1000000000)
                    },
                    constraints = { },
                    pose_stamped =
                    {
                        pose =
                        {
                            position = ikPose.position.To<FLU>(),
                            orientation = ikPose.rotation.To<FLU>()
                        }
                    },
                    robot_state =
                    {
                        joint_state =
                        {
                            name = robotKinematics.jointNames.ToArray(),
                            position = robotKinematics.jointArticulationBodies.Select(jointBody => 0d).ToArray()
                        }
                    }
                }
            };
            
            rosConnection.SendServiceMessage(serviceTopic, ikServiceRequest, resultsHandler);
        }

        #endregion
    }
}