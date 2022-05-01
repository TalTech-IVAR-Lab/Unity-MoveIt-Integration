namespace EE.TalTech.IVAR.ROS.MoveItIntegration
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Linq;
    using RosMessageTypes.Moveit;
    using RosMessageTypes.Shape;
    using UnityEngine;

    /// <summary>
    /// Marks the <see cref="GameObject"/> as a <see cref="CollisionObject"/> to be synchronized with MoveIt planning scene in ROS.
    /// </summary>
    public class MoveItCollisionObjectAlias : MonoBehaviour
    {
        #region Variables

        /// <summary>
        /// Unique identifier of this object.
        /// </summary>
        /// <remarks>
        /// Unique IDs are required so that MoveIt can tell the difference between the <see cref="CollisionObject"/>s synced from Unity.
        /// </remarks>
        public string UID { get; private set; }
        
        /// <summary>
        /// Cached list of all child colliders of this object.
        /// </summary>
        /// <remarks>
        /// These colliders are used by <see cref="MoveItPlanningSceneSynchronizer"/> when building a <see cref="CollisionObject"/> representation of this object for MoveIt.
        /// </remarks>
        public List<Collider> childColliders;
        
        #endregion

        #region Unity Callbacks

        private void Awake()
        {
            GenerateUID();
            CollectChildColliders();
        }

        #endregion

        #region Public Methods
        
        /// <summary>
        /// Collects all colliders which are children of this object.
        /// </summary>
        public void CollectChildColliders()
        {
            childColliders = GetComponentsInChildren<Collider>(true).ToList();
        }

        /// <summary>
        /// Generates <see cref="UID"/>.
        /// </summary>
        private void GenerateUID()
        {
            UID = $"{gameObject.name}_{(uint)GetHashCode()}";
        }

        #endregion
    }
}