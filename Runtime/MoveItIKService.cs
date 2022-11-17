namespace EE.TalTech.IVAR.Robotics.MoveItIntegration
{
    using System.Linq;
    using System.Security.Cryptography;
    using Cysharp.Threading.Tasks;
    using ROSIndustrial;
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

        /// <summary>
        /// Name of the planning group to get the IK solution for.
        /// </summary>
        public string planningGroupName = "manipulator";

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
        /// <returns>IK solver service response.</returns>
        public async UniTask<GetPositionIKResponse> ComputeIK(Pose targetWorldPose)
        {
            if (!isActiveAndEnabled)
            {
                Debug.LogWarning($"IK solution will not be computed because this {nameof(MoveItIKService)} component is disabled.", this);
                return null;
            }
        
            // Convert target pose to the robot's coordinate system
            var ikPose = new Pose
            {
                position = robotKinematics.RootLink.transform.InverseTransformPoint(targetWorldPose.position),
                rotation = Quaternion.Inverse(robotKinematics.RootLink.transform.rotation) * targetWorldPose.rotation
            };
            
            var rosPosition = ikPose.position.To<FLU>();
            var rosOrientation = ikPose.rotation.To<FLU>();

            var rosPose = new Pose
            {
                position = new Vector3
                {
                    x = rosPosition.x,
                    y = rosPosition.y,
                    z = rosPosition.z,
                },
                rotation = new Quaternion
                {
                    x = rosOrientation.x,
                    y = rosOrientation.y,
                    z = rosOrientation.z,
                    w = rosOrientation.w,
                }
            };
            
            Debug.LogWarning($"Getting solution for: {rosPose}");

            // Craft service request
            var ikServiceRequest = new GetPositionIKRequest
            {
                ik_request =
                {
                    avoid_collisions = true,
                    group_name = planningGroupName,
                    timeout =
                    {
                        sec = Mathf.FloorToInt(solutionTimeout),
                        nanosec = (uint) (solutionTimeout * 1000000000 % 1000000000)
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
            
            var response = await rosConnection.SendServiceMessage<GetPositionIKResponse>(serviceTopic, ikServiceRequest);
            return response;
        }

        #endregion
    }
}