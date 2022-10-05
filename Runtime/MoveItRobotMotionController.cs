namespace EE.TalTech.IVAR.Robotics.MoveItIntegration
{
    using System;
    using System.Collections.Generic;
    using System.Threading;
    using Cysharp.Threading.Tasks;
    using ROSIndustrial;
    using ROSIndustrial.Actions;
    using ROSIndustrial.MoveItIntegration;
    using RosMessageTypes.Industrial;
    using RosMessageTypes.Moveit;
    using RosMessageTypes.Std;
    using UnityEngine;
    using MoveGroupActionClient = ROSIndustrial.Actions.RosActionClient<
        RosMessageTypes.Moveit.MoveGroupActionGoal,
        RosMessageTypes.Moveit.MoveGroupActionFeedback,
        RosMessageTypes.Moveit.MoveGroupActionResult,
        RosMessageTypes.Moveit.MoveGroupGoal,
        RosMessageTypes.Moveit.MoveGroupFeedback,
        RosMessageTypes.Moveit.MoveGroupResult
    >;

    /// <summary>
    /// Interface for controlling the motion of a connected ROS-Industrial robot through MoveIt.
    /// </summary>
    public class MoveItRobotMotionController : RosConnectedBehaviour, IRosRobotMotionController
    {
        #region Variables (Public)

        [Header("Robot State Sources")]
        public RosIndustrialRobotStatusListener robotStatusListener;
        public RosIndustrialRobotJointStatesListener robotJointStatesListener;
        public UrdfRobotKinematicsDataProvider robotKinematics;
        
        /// <summary>
        /// MoveIt Move Group action topic.
        /// </summary>
        [Header("ROS Topics")]
        public string moveGroupActionTopic = "move_group";
        
        /// <summary>
        /// ROS topic to enable the robot.
        /// </summary>
        public string robotEnableServiceTopic = "robot_enable";

        /// <summary>
        /// ROS topic to disable the robot.
        /// </summary>
        public string robotDisableServiceTopic = "robot_disable";

        /// <summary>
        /// ROS topic to stop current robot motion.
        /// </summary>
        public string robotStopMotionServiceTopic = "stop_motion";

        /// <summary>
        /// Motion speed multiplier.
        /// </summary>
        [Header("Motion Planning Parameters")]
        [Range(0f, 1f)]
        public double velocityScalingFactor = 0.1d;

        /// <summary>
        /// Name of MoveIt's motion planning group to be used for trajectory execution.
        /// </summary>
        public string motionPlanningGroup = "manipulator";
        
        #endregion

        #region Variables (Public)

        /// <summary>
        /// Action goal ID of the last motion request.
        /// </summary>
        private string lastPlanningGoalID;

        private MoveGroupActionClient moveGroupActionClient;

        #endregion

        #region Unity Callbacks

        private void OnEnable()
        {
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotEnableServiceTopic);
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotDisableServiceTopic);
            rosConnection.RegisterRosService<StartMotionRequest, StopMotionResponse>(robotStopMotionServiceTopic);

            moveGroupActionClient = new MoveGroupActionClient(rosConnection, moveGroupActionTopic);
        }

        #endregion

        #region Public Methods (State)

        public void EnableNoWait() { EnableRobot().Forget(); }

        public void DisableNoWait() { DisableRobot().Forget(); }

        public async UniTask<TriggerResponse> EnableRobot()
        {
            var response = await rosConnection.SendServiceMessage<TriggerResponse>(robotEnableServiceTopic, new TriggerRequest());

            if (!response.success) { Debug.LogError($"Error when trying to enable robot: {response.message}", this); }
            else { Debug.Log("Robot enabled successfully.", this); }

            return response;
        }

        public async UniTask<TriggerResponse> DisableRobot()
        {
            var response = await rosConnection.SendServiceMessage<TriggerResponse>(robotDisableServiceTopic, new TriggerRequest());

            if (!response.success) { Debug.LogError($"Error when trying to disable robot: {response.message}", this); }
            else { Debug.Log("Robot disabled successfully.", this); }

            return response;
        }

        #endregion

        #region Public Methods (Motion)

        /// <summary>
        /// Robot's joints positions adjusted for Unity (angular joint positions in degrees, linear joint positions in meters).
        /// </summary>
        /// <returns>Arrays of joint names and positions.</returns>
        public (string[], double[]) GetJointPositions()
        {
            return ((string[])robotJointStatesListener.jointNames.Clone(), (double[])robotJointStatesListener.unityJointPositions.Clone());
        }

        public void StopMotionNoWait()
        {
            StopMotion().Forget();
        }

        public async UniTask<bool> StopMotion()
        {
            CancelLastPlanningGoal();
            return true;
        }

        public async UniTask<bool> Move(string[] jointNames, double[] positions, CancellationToken cancellationToken = default)
        {
            // If previous planning operation has not been completed, cancel it
            CancelLastPlanningGoal();

            double[] targetPositions = RobotJointPositionsConversionUtility.UnityToRos(robotKinematics, jointNames, positions);

            string unityPositions = string.Join(", ", positions);
            string rosPositions = string.Join(", ", targetPositions);
            Debug.Log("Planning motion to position:\n" +
                      $"{unityPositions} (Unity coordinate space)" +
                      $"{rosPositions} (ROS coordinate space)");

            // Craft goal joint constraints
            var goalJointConstraints = new List<JointConstraintMsg>();
            for (int i = 0; i < jointNames.Length; i++)
            {
                goalJointConstraints.Add(new JointConstraintMsg
                {
                    joint_name = jointNames[i],
                    position = targetPositions[i],
                    weight = 1
                });
            }

            // Prepare action goal message
            var goal = new MoveGroupGoal
            {
                request = new MotionPlanRequestMsg
                {
                    group_name = motionPlanningGroup,
                    max_velocity_scaling_factor = velocityScalingFactor,
                    goal_constraints = new[]
                    {
                        new ConstraintsMsg
                        {
                            joint_constraints = goalJointConstraints.ToArray()
                        }
                    }
                },
            };

            // Execute motion action
            string goalID = moveGroupActionClient.InitiateAction(goal);
            lastPlanningGoalID = goalID;

            await foreach (var status in moveGroupActionClient.MonitorActionStatus(goalID))
            {
                var code = new RosActionGoalStatusCode(status.status);
                Debug.Log($"[ID {goalID}] Motion status update: [{status.status}/{code}] {status.text}");
            }

            var result = await moveGroupActionClient.WaitUntilActionCompletes(goalID, cancellationToken);

            var statusCode = new RosActionGoalStatusCode(result.status.status);
            var moveItErrorCode = new MoveItErrorCode(result.result.error_code.val);
            if (!statusCode.IsSuccessful)
            {
                if (statusCode.value == RosActionGoalStatusCodeEnum.PREEMPTED)
                {
                    Debug.Log($"[ID {goalID}] Motion action was cancelled.");
                    return false;
                }

                Debug.LogError($"[ID {goalID}] Motion failed to be processed. Status code {statusCode.value} ({statusCode}):\n" +
                               $"{result.status.text} (MoveIt error code {moveItErrorCode.intValue}/{moveItErrorCode.name})");

                return false;
            }

            return true;
        }
        
        #endregion

        #region Internal Methods

        /// <summary>
        /// Cancels the last motion action if it is still running. 
        /// </summary>
        private void CancelLastPlanningGoal()
        {
            if (lastPlanningGoalID == null) return;
            if (moveGroupActionClient.IsActionCompleted(lastPlanningGoalID)) return;
            
            Debug.LogWarning($"Previous planning action is not complete yet (ID {lastPlanningGoalID}). It will be canceled.");
            moveGroupActionClient.CancelAction(lastPlanningGoalID);
        }

        #endregion
    }
}