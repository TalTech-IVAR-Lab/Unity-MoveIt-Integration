# Unity MoveIt Integration

Collection of scripts for interacting with ROS MoveIt motion planning library from Unity.

More detailed documentation [can be found here](https://jgroxz.notion.site/Unity-x-ROS-Wiki-c2c7bf7a271946d694bfa30a5aca4822).

## Installation

Add the package and its TalTech dependency directly to the Unity project's
`Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.cysharp.unitask": "https://github.com/Cysharp/UniTask.git?path=/src/UniTask/Assets/Plugins/UniTask#2.3.1",
    "ee.taltech.ivar.robotics.moveit": "https://github.com/TalTech-IVAR-Lab/Unity-MoveIt-Integration.git#v1.3.0",
    "ee.taltech.ivar.robotics.ros-industrial": "https://github.com/TalTech-IVAR-Lab/Unity-ROS-Industrial-Integration.git#v1.3.1",
    "io.extendreality.zinnia.unity": "https://github.com/ExtendRealityLtd/Zinnia.Unity.git#v2.0.0"
  }
}
```

Unity does not resolve version-only transitive dependencies from Git. Keep all
four entries in the project manifest unless another package source provides them.
