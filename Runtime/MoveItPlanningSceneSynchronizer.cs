namespace EE.TalTech.IVAR.ROS.MoveItIntegration
{
    using System.Collections.Generic;
    using Robotics.ROSIndustrial;
    using RosMessageTypes.Geometry;
    using RosMessageTypes.Moveit;
    using RosMessageTypes.ObjectRecognition;
    using RosMessageTypes.Octomap;
    using RosMessageTypes.Shape;
    using Unity.Robotics.ROSTCPConnector;
    using Unity.Robotics.ROSTCPConnector.ROSGeometry;
    using UnityEngine;
    using RosPose = RosMessageTypes.Geometry.PoseMsg;
    using RosPlane = RosMessageTypes.Shape.PlaneMsg;
    using RosMesh = RosMessageTypes.Shape.MeshMsg;

    /// <summary>
    /// Provides functionality for synchronizing objects in Unity scene with MoveIt planning scene in ROS.
    /// </summary>
    /// <remarks>
    /// Constructing MoveIt planning scene based on Unity scene allows to simplify the development and
    /// facilitate program planning. 
    /// </remarks>
    public class MoveItPlanningSceneSynchronizer : MonoBehaviour
    {
        #region Constants

        // Names of some Unity meshes to map to ROS primitive shapes
        private const string MESH_NAME_PLANE = "Plane";
        private const string MESH_NAME_CONE = "Cone";
        private const string MESH_NAME_CYLINDER = "Cylinder";
        
        private const string APPLY_PLANNING_SCENE_SERVICE_TOPIC = "apply_planning_scene";

        #endregion
        
        #region Variables
        
        public ROSConnection rosConnection;

        public string planningSceneName = "Default";

        public UrdfRobotKinematicsDataProvider robotData;

        /// <summary>
        /// List of <see cref="MoveItCollisionObjectAlias"/>s this synchronizer is responsible for.
        /// </summary>
        public List<MoveItCollisionObjectAlias> synchronizedObjects = new List<MoveItCollisionObjectAlias>();

        /// <summary>
        /// Set of UIDs of <see cref="MoveItCollisionObjectAlias"/>es tracked as of the last synchronization. 
        /// </summary>
        /// <remarks>
        /// This is used to decide what to do with each object during synchronization updates.
        /// Some objects have to be added to the scene anew, while others may just require a position update, and others have to be removed.
        /// </remarks>
        private HashSet<string> lastTrackedAliasesUIDs = new HashSet<string>();
        
        #endregion

        #region Public Methods

        public void Synchronize()
        {
            var request = new ApplyPlanningSceneRequest
            {
                scene =
                {
                    name = planningSceneName,
                    robot_state = new RobotStateMsg(), // TODO
                    robot_model_name = robotData.robot.name, // TODO
                    fixed_frame_transforms = new TransformStampedMsg[0],
                    allowed_collision_matrix = new AllowedCollisionMatrixMsg(),
                    link_padding = new LinkPaddingMsg[0],
                    link_scale = new LinkScaleMsg[0],
                    object_colors = new ObjectColorMsg[0],
                    world =
                    {
                        collision_objects = GetUpdatedCollisionObjects(),
                        octomap = new OctomapWithPoseMsg()
                    },
                    is_diff = true
                }
            };

            rosConnection.SendServiceMessage<ApplyPlanningSceneResponse>(APPLY_PLANNING_SCENE_SERVICE_TOPIC, request, HandleServiceResponse);
        }

        #endregion

        #region Internal Methods

        /// <summary>
        /// Callback to handle response from MoveIt scene service.
        /// </summary>
        /// <param name="response">Response.</param>
        private void HandleServiceResponse(ApplyPlanningSceneResponse response)
        {
            bool success = response.success;

            if (success) { Debug.Log("MoveIt scene synchronized successfully.", this); }
            else { Debug.LogError("Error when synchronizing MoveIt scene.", this); }
        }

        /// <summary>
        /// Converts <see cref="synchronizedObjects"/> to the array of <see cref="CollisionObject"/>s suitable for sending to MoveIt.
        /// </summary>
        private CollisionObjectMsg[] GetUpdatedCollisionObjects()
        {
            var collisionObjects = new List<CollisionObjectMsg>();

            // Check removed objects
            foreach (var lastUID in lastTrackedAliasesUIDs)
            {
                
            }
            
            foreach (var objectAlias in synchronizedObjects)
            {
                CollisionObjectMsg collisionObject = new CollisionObjectMsg();
                
                // TODO: handle other cases! action must be determined based on object state
                if (!lastTrackedAliasesUIDs.Contains(objectAlias.UID))
                {
                    // Newly added object, generate add operation
                    collisionObject = AddCollisionObjectFromAlias(objectAlias);
                }
                // TODO: objects which are not tracked anymore must be removed  
                // if NOT_IN_TRACKED or DESTROYED remove
                // TODO: objects which only changed their position must be moved
                // if MOVED move
                // TODO: objects which changed colliders must be re-synched <- this is performance heavy
                // if MODIFIED resync
                
                collisionObjects.Add(collisionObject);
            }

            return collisionObjects.ToArray();
        }

        /// <summary>
        /// Generates <see cref="CollisionObject"/> with ADD operation from the given <see cref="MoveItCollisionObjectAlias"/>.
        /// </summary>
        /// <param name="objectAlias"></param>
        /// <returns></returns>
        private CollisionObjectMsg AddCollisionObjectFromAlias(MoveItCollisionObjectAlias objectAlias)
        {
            var primitives = new List<SolidPrimitiveMsg>();
            var primitive_poses = new List<RosPose>();
            var planes = new List<RosPlane>();
            var plane_poses = new List<RosPose>();
            var meshes = new List<RosMesh>();
            var mesh_poses = new List<RosPose>();

            // Collect all colliders
            foreach (var childCollider in objectAlias.childColliders)
            {
                var origin = robotData.RootLink.transform;

                switch (childCollider)
                {
                    case BoxCollider boxCollider:
                        primitives.Add(BoxColliderToRosBoxPrimitive(boxCollider));
                        primitive_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        break;
                    case SphereCollider sphereCollider:
                        primitives.Add(SphereColliderToRosSpherePrimitive(sphereCollider));
                        primitive_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        break;
                    case CapsuleCollider capsuleCollider:
                        primitives.Add(CapsuleColliderToRosCylinderPrimitive(capsuleCollider));
                        primitive_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        break;
                    case MeshCollider meshCollider:
                        // TODO: cone
                        if (meshCollider.sharedMesh.name == MESH_NAME_CYLINDER)
                        {
                            primitives.Add(CylinderColliderToRosCylinderPrimitive(meshCollider));
                            primitive_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        }
                        else if (meshCollider.sharedMesh.name == MESH_NAME_PLANE)
                        {
                            planes.Add(PlaneColliderToRosPlane(meshCollider));
                            plane_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        }
                        else
                        {
                            meshes.Add(MeshColliderToRosMesh(meshCollider));
                            mesh_poses.Add(GetColliderPoseRelative(childCollider, origin));
                        }

                        break;
                    default:
                        Debug.LogError($"Cannot infer collider type from collider '{childCollider}' on tracked object '{objectAlias.name}'.", childCollider);
                        break;
                }
            }

            // Construct collision object
            var collisionObject = new CollisionObjectMsg()
            {
                header =
                {
                    frame_id = robotData.RootLink.name,
                },
                id = objectAlias.UID,
                type = new ObjectTypeMsg(),
                operation = CollisionObjectMsg.ADD, 
                primitives = primitives.ToArray(),
                primitive_poses = primitive_poses.ToArray(),
                planes = planes.ToArray(),
                plane_poses = plane_poses.ToArray(),
                meshes = meshes.ToArray(),
                mesh_poses = mesh_poses.ToArray(),
            };
            return collisionObject;
        }

        // private CollisionObjectMsg UpdateOperationFromAlias(MoveItCollisionObjectAlias objectAlias)
        // {
        //     
        // }
        
        // private CollisionObjectMsg RemoveOperationFromAliasUID(string UID)
        // {
        //     // TODO: detach collision object first
        //     return new CollisionObject
        //     {
        //         
        //     };
        // }

        #endregion

        #region Internal Methods (Colliders Conversions)

        /// <summary>
        /// Converts collider pose to ROS Pose relative to the given origin.
        /// </summary>
        /// <param name="collider">Collider which pose has to be converted.</param>
        /// <param name="origin"><see cref="UnityEngine.Transform"/> relative to which the returned pose must be calculated.</param>
        /// <returns>Collider's ROS pose.</returns>
        private static RosPose GetColliderPoseRelative(Collider collider, Transform origin)
        {
            // Find center offset (this can be non-zero only on primitive Unity colliders)
            var colliderCenterLocalOffset = collider switch
            {
                BoxCollider boxCollider => boxCollider.center,
                SphereCollider sphereCollider => sphereCollider.center,
                CapsuleCollider capsuleCollider => capsuleCollider.center,
                _ => Vector3.zero
            };

            // Find collider world pose
            var colliderWorldPose = new Pose
            {
                position = collider.transform.TransformPoint(colliderCenterLocalOffset),
                rotation = collider.transform.rotation
            };

            // Find collider pose in origin coordinates
            var colliderOriginPose = new Pose
            {
                position = origin.InverseTransformPoint(colliderWorldPose.position),
                rotation = Quaternion.Inverse(origin.rotation) * colliderWorldPose.rotation
            };

            // Convert to ROS coordinates
            return new RosPose
            {
                position = colliderOriginPose.position.To<FLU>(),
                orientation = colliderOriginPose.rotation.To<FLU>()
            };
        }

        /// <summary>
        /// Converts Unity <see cref="BoxCollider"/> to ROS <see cref="SolidPrimitive"/> box.
        /// </summary>
        /// <param name="boxCollider">Box collider to convert.</param>
        /// <returns>ROS box primitive.</returns>
        private static SolidPrimitiveMsg BoxColliderToRosBoxPrimitive(BoxCollider boxCollider)
        {
            // Convert box size to ROS coordinates (and make sure it is positive across all axes)
            var size = boxCollider.size.To<FLU>();
            size = new Vector3<FLU>(Mathf.Abs(size.x), Mathf.Abs(size.y), Mathf.Abs(size.z));
            var scale = boxCollider.transform.lossyScale.To<FLU>();
            scale = new Vector3<FLU>(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
            size.Scale(scale);
            
            return new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.BOX,
                dimensions = new double[] {size.x, size.y, size.z}
            };
        }

        /// <summary>
        /// Converts Unity <see cref="SphereCollider"/> to ROS <see cref="SolidPrimitive"/> sphere.
        /// </summary>
        /// <param name="sphereCollider">Sphere collider to convert.</param>
        /// <returns>ROS sphere primitive.</returns>
        private static SolidPrimitiveMsg SphereColliderToRosSpherePrimitive(SphereCollider sphereCollider)
        {
            float radius = sphereCollider.radius;
            var scale = sphereCollider.transform.lossyScale.To<FLU>();
            scale = new Vector3<FLU>(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
            // TODO: add warning if sphere scale is non-uniform - it cannot be "squished" in MoveIt like in Unity, so we have to pick the max value from all scale axes
            radius *= Mathf.Max(scale.x, scale.y, scale.z);
            
            return new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.SPHERE,
                dimensions = new double[] {radius}
            };
        }

        /// <summary>
        /// Converts Unity <see cref="CapsuleCollider"/> to ROS <see cref="SolidPrimitive"/> cylinder.
        /// </summary>
        /// <param name="capsuleCollider">Capsule collider to convert.</param>
        /// <returns>ROS box primitive.</returns>
        private static SolidPrimitiveMsg CapsuleColliderToRosCylinderPrimitive(CapsuleCollider capsuleCollider)
        {
            // TODO: should we treat capsule colliders as cylinders?

            var scale = capsuleCollider.transform.lossyScale;
            float height = 0;
            float radius = 0;
            
            // TODO: how should we treat capsule direction? ROS cylinder is always oriented along Z (up) axis
            switch (capsuleCollider.direction)
            {
                case 0:
                    Debug.LogError($"X-oriented capsules are not yet supported in {nameof(MoveItPlanningSceneSynchronizer)}. We need to figure out an elegant way to map them to ROS.", capsuleCollider);
                    height = 0;
                    radius = 0;
                    break;
                case 1:
                    height = capsuleCollider.height * scale.y;
                    radius = capsuleCollider.radius * scale.x;
                    break;
                case 2:
                    Debug.LogError($"Z-oriented capsules are not yet supported in {nameof(MoveItPlanningSceneSynchronizer)}. We need to figure out an elegant way to map them to ROS.", capsuleCollider);
                    height = 0;
                    radius = 0;
                    break;
            }
            
            return new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.CYLINDER,
                dimensions = new double[] {height, radius}
            };
        }

        /// <summary>
        /// Converts Unity cylinder <see cref="MeshCollider"/> to ROS <see cref="SolidPrimitive"/> cylinder.
        /// </summary>
        /// <param name="cylinderCollider">Cylinder collider to convert.</param>
        /// <returns>ROS cylinder primitive.</returns>
        private static SolidPrimitiveMsg CylinderColliderToRosCylinderPrimitive(MeshCollider cylinderCollider)
        {
            var scale = cylinderCollider.transform.lossyScale.To<FLU>();
            scale = new Vector3<FLU>(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
            float height = scale.y * 2;
            float radius = scale.x / 2;
            
            return new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.CYLINDER,
                dimensions = new double[] {height, radius}
            };
        }
        
        /// <summary>
        /// Converts Unity plane <see cref="MeshCollider"/> to <see cref="RosMessageTypes.Shape.Plane"/>.
        /// </summary>
        /// <remarks>
        /// Based on http://answers.unity.com/answers/638879/view.html.
        /// </remarks>
        /// <param name="planeCollider">Plane collider to convert.</param>
        /// <returns>Converted ROS plane.</returns>
        private static RosPlane PlaneColliderToRosPlane(MeshCollider planeCollider)
        {
            var planeTransform = planeCollider.transform;
            var planeUpRos = planeTransform.up.To<FLU>();

            float a = planeUpRos.x;
            float b = planeUpRos.y;
            float c = planeUpRos.z;
            float d = -Vector3<FLU>.Dot(planeUpRos, Vector3<FLU>.zero);

            return new RosPlane
            {
                coef = new double[] {a, b, c, d}
            };
        }

        /// <summary>
        /// Converts Unity <see cref="MeshCollider"/> to Ros <see cref="RosMessageTypes.Shape.Mesh"/>.
        /// </summary>
        /// <param name="meshCollider">Mesh collider to convert.</param>
        /// <returns>Ros mesh.</returns>
        // TODO: figure out how generated convex colliders can be supported
        private static RosMesh MeshColliderToRosMesh(MeshCollider meshCollider)
        {
            var mesh = meshCollider.sharedMesh;

            if (!mesh.isReadable)
            {
                Debug.LogError($"Cannot convert mesh '{mesh.name}' to ROS Mesh as it is not Read/Write enabled. Please make the mesh asset readable to fix this issue.", mesh);
                return new RosMesh();
            }

            // Construct triangles array
            var rosTriangles = new MeshTriangleMsg[mesh.triangles.Length / 3];
            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                uint x = (uint) mesh.triangles[i];
                uint y = (uint) mesh.triangles[i + 1];
                uint z = (uint) mesh.triangles[i + 2];

                int triangle_index = i / 3;
                rosTriangles[triangle_index] = new MeshTriangleMsg(new[] {x, y, z});
            }

            // Construct vertices array
            var rosVertices = new PointMsg[mesh.vertexCount];
            for (int i = 0; i < mesh.vertexCount; i++)
            {
                var rosVertex = mesh.vertices[i].To<FLU>();
                rosVertex.Scale(meshCollider.transform.lossyScale.To<FLU>());
                rosVertices[i] = rosVertex;
            }

            return new RosMesh
            {
                triangles = rosTriangles,
                vertices = rosVertices
            };
        }

        #endregion
    }
}