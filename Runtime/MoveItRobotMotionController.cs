namespace EE.TalTech.IVAR.Robotics.MoveItIntegration
{
    using System.Collections.Generic;
    using System.Threading;
    using Cysharp.Threading.Tasks;
    using ROSIndustrial;
    using ROSIndustrial.Actions;
    using ROSIndustrial.MoveItIntegration;
    using RosMessageTypes.Geometry;
    using RosMessageTypes.Industrial;
    using RosMessageTypes.Moveit;
    using RosMessageTypes.Shape;
    using RosMessageTypes.Std;
    using Unity.Robotics.ROSTCPConnector.ROSGeometry;
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
        [Range(0.0001f, 1f)]
        public double velocityScalingFactor = 0.1d;

        /// <summary>
        /// Motion acceleration multiplier.
        /// </summary>
        [Range(0.0001f, 1f)]
        public double accelerationScalingFactor = 1d;

        /// <summary>
        /// Name of MoveIt's motion planning group to be used for trajectory execution.
        /// </summary>
        public string motionPlanningGroup = "manipulator";

        #endregion

        #region Variables (Private)

        /// <summary>
        /// Action goal ID of the last motion request.
        /// </summary>
        private string lastPlanningGoalID;

        private MoveGroupActionClient moveGroupActionClient;

        /// <summary>
        /// Transform used for 
        /// </summary>
        private Transform cartesianRequestReferenceTransform;

        #endregion

        #region Unity Callbacks

        private void Start()
        {
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotEnableServiceTopic);
            rosConnection.RegisterRosService<TriggerRequest, TriggerResponse>(robotDisableServiceTopic);
            rosConnection.RegisterRosService<StartMotionRequest, StopMotionResponse>(robotStopMotionServiceTopic);

            moveGroupActionClient = new MoveGroupActionClient(rosConnection, moveGroupActionTopic);

            if (!cartesianRequestReferenceTransform)
            {
                cartesianRequestReferenceTransform = new GameObject("Cartesian Request Reference Transform").transform;
                cartesianRequestReferenceTransform.gameObject.hideFlags = HideFlags.HideInHierarchy;
            }
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
        public (string[], double[]) GetJointPositions() { return ((string[])robotJointStatesListener.jointNames.Clone(), (double[])robotJointStatesListener.unityJointPositions.Clone()); }

        public void StopMotionNoWait() { StopMotion().Forget(); }

        public async UniTask<bool> StopMotion()
        {
            CancelLastPlanningGoal();
            return true;
        }

        public void MoveToZeroNoWait() { MoveToZero().Forget(); }

        public async UniTask<bool> MoveToZero()
        {
            string[] names = robotKinematics.jointNames.ToArray();
            double[] zeros = new double[robotKinematics.joints.Count];
            zeros[4] = 90f;  // temporary home position fix for the experiments (TODO: remove)
            return await Move(names, zeros);
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
                    pipeline_id = "pilz_industrial_motion_planner",
                    planner_id = "PTP",
                    group_name = motionPlanningGroup,
                    max_velocity_scaling_factor = velocityScalingFactor,
                    max_acceleration_scaling_factor = accelerationScalingFactor,
                    goal_constraints = new[]
                    {
                        new ConstraintsMsg
                        {
                            joint_constraints = goalJointConstraints.ToArray()
                        }
                    },
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
        
        /// <summary>
        /// Moves robot's end-effector link to the given pose.
        /// </summary>
        /// <param name="targetWorldPose"></param>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        public async UniTask<bool> MoveCartesian(Pose targetWorldPose, CancellationToken cancellationToken = default)
        {
            // If previous planning operation has not been completed, cancel it
            CancelLastPlanningGoal();

            // Convert pose from world space to robot's local space
            var baseLinkTransform = robotKinematics.RootLink.transform;
            
            cartesianRequestReferenceTransform.SetParent(baseLinkTransform);
            cartesianRequestReferenceTransform.localPosition = Vector3.zero;
            cartesianRequestReferenceTransform.rotation = targetWorldPose.rotation;

            var targetPosition = cartesianRequestReferenceTransform.InverseTransformPoint(targetWorldPose.position);
            var rosPosition = targetPosition.To<FLU>();
            rosPosition *= -1f;

            var rosOrientation = cartesianRequestReferenceTransform.rotation.To<FLU>();
            
            // 

            string baseLinkName = robotKinematics.RootLink.name;
            string endEffectorLinkName = "tool0"; //robotKinematics.links.Last().name;

            double goalWeight = 1;

            Debug.Log($"Planning cartesian motion for end-effector link '{endEffectorLinkName}' to {rosPosition} {rosOrientation}.");

            // Volume
            var positionConstraintRegion = new BoundingVolumeMsg
            {
                primitive_poses = new[]
                {
                    new PoseMsg
                    {
                        position = new PointMsg(0d, 0d, 0d)
                    }
                },
                primitives = new[]
                {
                    new SolidPrimitiveMsg
                    {
                        type = SolidPrimitiveMsg.SPHERE,
                        dimensions = new[] { 100000000d }
                    }
                }
            };

            // Prepare action goal message
            var goal = new MoveGroupGoal
            {
                request = new MotionPlanRequestMsg
                {
                    pipeline_id = "pilz_industrial_motion_planner",
                    planner_id = "LIN",
                    group_name = motionPlanningGroup,
                    max_velocity_scaling_factor = velocityScalingFactor,
                    max_acceleration_scaling_factor = accelerationScalingFactor,
                    goal_constraints = new[]
                    {
                        new ConstraintsMsg
                        {
                            position_constraints = new[]
                            {
                                new PositionConstraintMsg
                                {
                                    header = new HeaderMsg
                                    {
                                        frame_id = baseLinkName
                                    },
                                    link_name = endEffectorLinkName,
                                    target_point_offset = rosPosition,
                                    constraint_region = positionConstraintRegion,
                                    weight = goalWeight
                                }
                            },
                            orientation_constraints = new[]
                            {
                                new OrientationConstraintMsg
                                {
                                    link_name = endEffectorLinkName,
                                    orientation = rosOrientation,
                                    weight = 0.01d // goalWeight
                                }
                            }
                        }
                    },
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

        // TODO: WIP
        // /// <summary>
        // /// Move robot using cartesian coordinates.
        // /// </summary>
        // /// <param name="pose"></param>
        // /// <param name="cancellationToken"></param>
        // /// <returns></returns>
        // public async UniTask<bool> MoveCartesian(Pose pose, CancellationToken cancellationToken = default)
        // {
        //     // If previous planning operation has not been completed, cancel it
        //     CancelLastPlanningGoal();
        //     
        //     // Convert target pose to the robot's coordinate system
        //     var ikPose = new Pose
        //     {
        //         position = robotKinematics.RootLink.transform.InverseTransformPoint(pose.position),
        //         rotation = Quaternion.Inverse(robotKinematics.RootLink.transform.rotation) * pose.rotation
        //     };
        //     
        //     // Convert target pose to ROS coordinate space
        //     
        //     
        //     string unityPositions = string.Join(", ", positions);
        //     string rosPositions = string.Join(", ", targetPositions);
        //     Debug.Log("Planning motion to position:\n" +
        //               $"{unityPositions} (Unity coordinate space)" +
        //               $"{rosPositions} (ROS coordinate space)");
        //
        //     // Craft goal joint constraints
        //     var goalJointConstraints = new List<JointConstraintMsg>();
        //     for (int i = 0; i < jointNames.Length; i++)
        //     {
        //         goalJointConstraints.Add(new JointConstraintMsg
        //         {
        //             joint_name = jointNames[i],
        //             position = targetPositions[i],
        //             weight = 1
        //         });
        //     }
        //
        //     // Prepare action goal message
        //     var goal = new MoveGroupGoal
        //     {
        //         request = new MotionPlanRequestMsg
        //         {
        //             group_name = motionPlanningGroup,
        //             max_velocity_scaling_factor = velocityScalingFactor,
        //             goal_constraints = new[]
        //             {
        //                 new ConstraintsMsg
        //                 {
        //                     joint_constraints = goalJointConstraints.ToArray()
        //                 }
        //             },
        //         },
        //     };
        //
        //     // Execute motion action
        //     string goalID = moveGroupActionClient.InitiateAction(goal);
        //     lastPlanningGoalID = goalID;
        //
        //     await foreach (var status in moveGroupActionClient.MonitorActionStatus(goalID))
        //     {
        //         var code = new RosActionGoalStatusCode(status.status);
        //         Debug.Log($"[ID {goalID}] Motion status update: [{status.status}/{code}] {status.text}");
        //     }
        //
        //     var result = await moveGroupActionClient.WaitUntilActionCompletes(goalID, cancellationToken);
        //
        //     var statusCode = new RosActionGoalStatusCode(result.status.status);
        //     var moveItErrorCode = new MoveItErrorCode(result.result.error_code.val);
        //     if (!statusCode.IsSuccessful)
        //     {
        //         if (statusCode.value == RosActionGoalStatusCodeEnum.PREEMPTED)
        //         {
        //             Debug.Log($"[ID {goalID}] Motion action was cancelled.");
        //             return false;
        //         }
        //
        //         Debug.LogError($"[ID {goalID}] Motion failed to be processed. Status code {statusCode.value} ({statusCode}):\n" +
        //                        $"{result.status.text} (MoveIt error code {moveItErrorCode.intValue}/{moveItErrorCode.name})");
        //
        //         return false;
        //     }
        //
        //     return true;
        // }

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