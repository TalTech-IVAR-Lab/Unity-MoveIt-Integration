namespace EE.TalTech.IVAR.Robotics.ROSIndustrial
{
    using Cysharp.Threading.Tasks;
    using Robotics.MoveItIntegration;
    using UnityEngine;

    /// <summary>
    /// Mirrors joint positions of one robot to another.
    /// </summary>
    public class ControllerMirrorTest : MonoBehaviour
    {
        [SerializeReference]
        public MoveItRobotMotionController from;

        [SerializeReference]
        public UrdfRobotMotionController to;

        private void FixedUpdate()
        {
            (string[] names, double[] positions) = from.GetJointPositions();
            to.Move(names, positions).Forget();
        }
    }
}