namespace EE.TalTech.IVAR.ROS.MoveItIntegration
{
    /// <summary>
    /// Stores information about synchronization of a single <see cref="MoveItCollisionObjectAlias"/> for a single <see cref="MoveItPlanningSceneSynchronizer"/>. 
    /// </summary>
    public class MoveItCollisionObjectAliasSynchronizationState
    {
        public MoveItCollisionObjectAlias objectAlias;

        public MoveItCollisionObjectAliasSynchronizationState(MoveItCollisionObjectAlias objectAlias) { }

        public bool IsValid => false;
    }
}