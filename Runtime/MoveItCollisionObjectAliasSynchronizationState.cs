namespace EE.TalTech.IVAR.ROS.MoveItIntegration
{
    using System.Collections;
    using System.Collections.Generic;
    using UnityEngine;

    /// <summary>
    /// Stores information about synchronization of a single <see cref="MoveItCollisionObjectAlias"/> for a single <see cref="MoveItPlanningSceneSynchronizer"/>. 
    /// </summary>
    public class MoveItCollisionObjectAliasSynchronizationState
    {
        public MoveItCollisionObjectAlias objectAlias;
        
        public MoveItCollisionObjectAliasSynchronizationState(MoveItCollisionObjectAlias objectAlias)
        {
            
        }

        public bool IsValid => false;
    }
}