namespace EE.TalTech.IVAR.Robotics.MoveItIntegration.Utils
{
    using Cysharp.Threading.Tasks;
    using ROSIndustrial;
    using RosMessageTypes.Moveit;
    using UnityEngine;

    /// <summary>
    /// Provides functionality to compute motion plans for the connected robot through MoveIt planning service running in ROS. 
    /// </summary>
    public class MoveItMotionPlanningService : RosConnectedBehaviour
    {
        #region Variables

        /// <summary>
        /// ROS topic of the IK service.
        /// </summary>
        public string serviceTopic = "plan_kinematic_path";

        /// <summary>
        /// Robot kinematics data.
        /// </summary>
        public UrdfRobotKinematicsDataProvider robotKinematics;

        /// <summary>
        /// Maximum amount of time to allow for motion planner to produce a solution.
        /// Computation is aborted if this time limit is reached.
        /// </summary>
        /// <remarks>
        /// This does not account for network latency between Unity and ROS, so the actual time of aborted request may be longer.
        /// </remarks>
        public float solutionTimeout = 1.0f;

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
        /// <returns>Motion planning service response.</returns>
        public async UniTask<GetMotionPlanResponse> ComputeMotionPlan(Pose targetWorldPose)
        {
            if (!isActiveAndEnabled)
            {
                Debug.LogWarning($"IK solution will not be computed because this {nameof(MoveItIKService)} component is disabled.", this);
                return null;
            }

            // Craft service request
            var motionPlanningRequest = new MotionPlanRequestMsg()
            {
                goal_constraints = new ConstraintsMsg[]
                {
                    new ConstraintsMsg
                        { }
                },
                allowed_planning_time = solutionTimeout,
            };

            var response = await rosConnection.SendServiceMessage<GetMotionPlanResponse>(serviceTopic, motionPlanningRequest);
            return response;
        }

        #endregion
    }
}