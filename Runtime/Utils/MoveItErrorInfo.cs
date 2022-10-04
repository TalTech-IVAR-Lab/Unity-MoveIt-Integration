namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.MoveItIntegration
{
    using Common;

    /// <summary>
    /// Enum containing MoveIt error codes.
    /// </summary>
    /// <remarks>
    /// http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/MoveItErrorCodes.html
    /// </remarks>
    public enum MoveItErrorCodeEnum
    {
        // overall behavior
        SUCCESS = 1,
        FAILURE = 99999,
        PLANNING_FAILED = -1,
        INVALID_MOTION_PLAN = -2,
        MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3,
        CONTROL_FAILED = -4,
        UNABLE_TO_AQUIRE_SENSOR_DATA = -5,
        TIMED_OUT = -6,
        PREEMPTED = -7,

        // planning & kinematics request errors
        START_STATE_IN_COLLISION = -10,
        START_STATE_VIOLATES_PATH_CONSTRAINTS = -11,
        GOAL_IN_COLLISION = -12,
        GOAL_VIOLATES_PATH_CONSTRAINTS = -13,
        GOAL_CONSTRAINTS_VIOLATED = -14,
        INVALID_GROUP_NAME = -15,
        INVALID_GOAL_CONSTRAINTS = -16,
        INVALID_ROBOT_STATE = -17,
        INVALID_LINK_NAME = -18,
        INVALID_OBJECT_NAME = -19,

        // system errors
        FRAME_TRANSFORM_FAILURE = -21,
        COLLISION_CHECKING_UNAVAILABLE = -22,
        ROBOT_STATE_STALE = -23,
        SENSOR_INFO_STALE = -24,

        // kinematics errors
        NO_IK_SOLUTION = -31,
    }

    /// <summary>
    /// Helper class for converting MoveIt error codes into readable strings.
    /// </summary>
    public class MoveItErrorCode : StatusCodeBase<MoveItErrorCodeEnum>
    {
        public MoveItErrorCode(int value) : base(value) { }
    }
}