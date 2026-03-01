# Unity Integration for Robot Visualization

## Introduction

While Gazebo excels at physics simulation, **Unity** provides photorealistic rendering, advanced lighting, and sophisticated human-robot interaction scenarios. Unity's game engine capabilities make it ideal for:

- **High-fidelity visualization**: Realistic materials, lighting, and post-processing
- **Human-in-the-loop simulation**: Interactive scenarios with virtual humans
- **VR/AR integration**: Immersive robot teleoperation and training
- **UI/UX prototyping**: Testing robot interfaces before hardware deployment
- **Marketing and demos**: Professional-quality visualizations

## Unity Robotics Hub

NVIDIA and Unity Technologies provide the **Unity Robotics Hub**, which bridges ROS 2 and Unity for seamless integration.

### Architecture Overview

```
ROS 2 Network ←→ ROS-TCP-Connector ←→ Unity Scene
                  (TCP/IP Bridge)
```

**Components:**
- **ROS-TCP-Endpoint**: ROS 2 package that runs a TCP server
- **ROS-TCP-Connector**: Unity package that connects to the endpoint
- **URDF Importer**: Converts URDF files to Unity GameObjects
- **Message Generation**: Auto-generates C# classes from ROS messages

## Installation and Setup

### Prerequisites

- **Unity Hub**: Download from unity.com
- **Unity Editor**: Version 2021.3 LTS or newer
- **ROS 2 Humble**: Installed on Ubuntu 22.04 or WSL2

### Step 1: Install ROS-TCP-Endpoint

```bash
# Create workspace
mkdir -p ~/unity_ros_ws/src
cd ~/unity_ros_ws/src

# Clone ROS-TCP-Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/unity_ros_ws
colcon build
source install/setup.bash
```

### Step 2: Create Unity Project

1. Open Unity Hub
2. Create new 3D project (URP - Universal Render Pipeline recommended)
3. Open Package Manager (Window → Package Manager)
4. Add packages from git URL:
   - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

### Step 3: Configure ROS Settings

1. In Unity: **Robotics → ROS Settings**
2. Set ROS IP Address: `127.0.0.1` (or your ROS machine IP)
3. Set ROS Port: `10000`
4. Protocol: `ROS 2`

### Step 4: Start ROS-TCP-Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Importing URDF into Unity

### Method 1: URDF Importer (Recommended)

1. Place your URDF file and meshes in `Assets/URDF/`
2. In Unity: **Assets → Import Robot from URDF**
3. Select your URDF file
4. Configure import settings:
   - **Axis Type**: Y-Axis (Unity) or Z-Axis (ROS)
   - **Mesh Decomposer**: VHACD (for complex collision meshes)
5. Click **Import**

The importer creates:
- GameObject hierarchy matching URDF structure
- ArticulationBody components for joints
- Colliders and visual meshes
- Material assignments

### Method 2: Manual Import

For simple robots or custom control:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class SimpleRobotController : MonoBehaviour
{
    private ArticulationBody[] joints;

    void Start()
    {
        // Get all articulation bodies (joints)
        joints = GetComponentsInChildren<ArticulationBody>();

        // Subscribe to ROS topic
        ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Sensor.JointState>(
            "/joint_states",
            UpdateJointStates
        );
    }

    void UpdateJointStates(RosMessageTypes.Sensor.JointState msg)
    {
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }
}
```

## ROS 2 Communication in Unity

### Publishing Messages

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocityPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        // Get input
        float linear = Input.GetAxis("Vertical") * 2.0f;
        float angular = Input.GetAxis("Horizontal") * 2.0f;

        // Create message
        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg { x = linear, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = angular }
        };

        // Publish
        ros.Publish(topicName, msg);
    }
}
```

### Subscribing to Topics

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LaserScanVisualizer : MonoBehaviour
{
    private LineRenderer lineRenderer;

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();

        ROSConnection.GetOrCreateInstance().Subscribe<LaserScanMsg>(
            "/scan",
            VisualizeScan
        );
    }

    void VisualizeScan(LaserScanMsg msg)
    {
        lineRenderer.positionCount = msg.ranges.Length;

        for (int i = 0; i < msg.ranges.Length; i++)
        {
            float angle = msg.angle_min + i * msg.angle_increment;
            float range = msg.ranges[i];

            Vector3 point = new Vector3(
                range * Mathf.Cos(angle),
                0,
                range * Mathf.Sin(angle)
            );

            lineRenderer.SetPosition(i, point);
        }
    }
}
```

### Calling Services

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ServiceCaller : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<SetBoolRequest, SetBoolResponse>("/enable_autonomous");
    }

    public void EnableAutonomous(bool enable)
    {
        SetBoolRequest request = new SetBoolRequest { data = enable };

        ros.SendServiceMessage<SetBoolResponse>(
            "/enable_autonomous",
            request,
            OnServiceResponse
        );
    }

    void OnServiceResponse(SetBoolResponse response)
    {
        Debug.Log($"Service response: {response.message}");
    }
}
```

## Camera and Sensor Simulation

### RGB Camera

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera sensorCamera;
    public string topicName = "/camera/image_raw";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    public float publishRate = 30f;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        renderTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        sensorCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= 1f / publishRate)
        {
            timer = 0;
            PublishImage();
        }
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, resolutionWidth, resolutionHeight), 0, 0);
        texture2D.Apply();

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "camera_link"
            },
            height = (uint)resolutionHeight,
            width = (uint)resolutionWidth,
            encoding = "rgb8",
            step = (uint)(resolutionWidth * 3),
            data = texture2D.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

### Depth Camera

```csharp
public class DepthCameraPublisher : MonoBehaviour
{
    public Camera depthCamera;
    private Shader depthShader;

    void Start()
    {
        // Use Unity's depth shader
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
        depthShader = Shader.Find("Hidden/DepthToColor");

        // Configure camera for depth
        depthCamera.backgroundColor = Color.white;
        depthCamera.clearFlags = CameraClearFlags.SolidColor;
    }

    // Similar publishing logic as RGB camera
    // but encode depth values in the image data
}
```

## Lighting and Materials

### High-Quality Rendering Setup

```csharp
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class RenderingSetup : MonoBehaviour
{
    void Start()
    {
        // Enable HDR
        Camera.main.allowHDR = true;

        // Configure post-processing
        var volume = gameObject.AddComponent<Volume>();
        volume.isGlobal = true;

        var profile = ScriptableObject.CreateInstance<VolumeProfile>();

        // Add bloom
        var bloom = profile.Add<Bloom>();
        bloom.intensity.value = 0.3f;

        // Add ambient occlusion
        var ao = profile.Add<AmbientOcclusion>();
        ao.intensity.value = 0.5f;

        volume.profile = profile;
    }
}
```

### PBR Materials for Robots

Create realistic robot materials using Unity's Standard Shader:

1. **Metallic surfaces** (robot chassis):
   - Albedo: Dark gray
   - Metallic: 0.9
   - Smoothness: 0.7

2. **Plastic parts**:
   - Albedo: Color of choice
   - Metallic: 0.0
   - Smoothness: 0.5

3. **Rubber (tires, grips)**:
   - Albedo: Black
   - Metallic: 0.0
   - Smoothness: 0.2

## Human-Robot Interaction

### Virtual Human Characters

```csharp
using UnityEngine;
using UnityEngine.AI;

public class VirtualHuman : MonoBehaviour
{
    private NavMeshAgent agent;
    private Animator animator;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();
    }

    public void WalkTo(Vector3 destination)
    {
        agent.SetDestination(destination);
        animator.SetBool("Walking", true);
    }

    void Update()
    {
        // Stop animation when reached destination
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            animator.SetBool("Walking", false);
        }
    }
}
```

### Gesture Recognition

```csharp
public class GestureDetector : MonoBehaviour
{
    public Transform humanHand;
    public Transform robotCamera;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Std.StringMsg>("/detected_gesture");
    }

    void Update()
    {
        // Simple wave detection
        if (IsWaving())
        {
            var msg = new RosMessageTypes.Std.StringMsg { data = "wave" };
            ros.Publish("/detected_gesture", msg);
        }
    }

    bool IsWaving()
    {
        // Check if hand is in camera view and moving
        Vector3 viewPos = robotCamera.GetComponent<Camera>().WorldToViewportPoint(humanHand.position);
        bool inView = viewPos.x > 0 && viewPos.x < 1 && viewPos.y > 0 && viewPos.y < 1 && viewPos.z > 0;

        // Add velocity check for actual waving motion
        return inView && humanHand.GetComponent<Rigidbody>().velocity.magnitude > 1.0f;
    }
}
```

## VR Integration for Teleoperation

### VR Controller Mapping

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRTeleoperation : MonoBehaviour
{
    private ROSConnection ros;
    private InputDevice rightController;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Geometry.PoseMsg>("/target_pose");

        rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
    }

    void Update()
    {
        if (rightController.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position) &&
            rightController.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation))
        {
            // Publish controller pose as robot target
            var msg = new RosMessageTypes.Geometry.PoseMsg
            {
                position = new RosMessageTypes.Geometry.PointMsg
                {
                    x = position.x,
                    y = position.y,
                    z = position.z
                },
                orientation = new RosMessageTypes.Geometry.QuaternionMsg
                {
                    x = rotation.x,
                    y = rotation.y,
                    z = rotation.z,
                    w = rotation.w
                }
            };

            ros.Publish("/target_pose", msg);
        }
    }
}
```

## Performance Optimization

### Best Practices

1. **LOD (Level of Detail)**: Use LOD groups for complex meshes
2. **Occlusion Culling**: Enable for large environments
3. **Batching**: Combine static meshes to reduce draw calls
4. **Texture Atlasing**: Combine multiple textures
5. **Physics Optimization**: Use simplified collision meshes

```csharp
public class PerformanceManager : MonoBehaviour
{
    void Start()
    {
        // Set target frame rate
        Application.targetFrameRate = 60;

        // Enable occlusion culling
        Camera.main.useOcclusionCulling = true;

        // Optimize physics
        Physics.defaultSolverIterations = 6;
        Physics.defaultSolverVelocityIterations = 1;
    }
}
```

## Practical Example: Humanoid Visualization

### Complete Scene Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class HumanoidVisualization : MonoBehaviour
{
    public GameObject humanoidPrefab;
    private GameObject humanoidInstance;
    private ArticulationBody[] joints;
    private ROSConnection ros;

    void Start()
    {
        // Spawn humanoid
        humanoidInstance = Instantiate(humanoidPrefab, Vector3.zero, Quaternion.identity);
        joints = humanoidInstance.GetComponentsInChildren<ArticulationBody>();

        // Setup ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<RosMessageTypes.Sensor.JointStateMsg>("/joint_states", UpdateJoints);
        ros.RegisterPublisher<RosMessageTypes.Sensor.ImageMsg>("/unity/camera/image");

        // Setup camera publisher
        gameObject.AddComponent<CameraPublisher>();
    }

    void UpdateJoints(RosMessageTypes.Sensor.JointStateMsg msg)
    {
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }
}
```

## Debugging and Tools

### Unity Console Integration

```csharp
public class ROSDebugger : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().ListenForTopics(
            (topics) => {
                Debug.Log($"Available topics: {string.Join(", ", topics)}");
            }
        );
    }
}
```

### ROS Bridge Monitor

```bash
# Monitor Unity-ROS connection
ros2 topic list | grep unity
ros2 topic echo /unity/camera/image --no-arr
```

## Exercises

1. **Basic Integration**: Import a simple robot URDF into Unity and control it from ROS 2.

2. **Camera Streaming**: Set up a Unity camera and visualize the stream in RViz.

3. **Interactive Scene**: Create a scene with obstacles and a virtual human, then navigate a robot using ROS 2 navigation stack.

4. **VR Teleoperation**: Implement basic VR controller mapping to control a robot arm.

5. **Multi-Robot Visualization**: Spawn multiple robots in Unity and control them independently from ROS 2.

## Summary

- **Unity** provides photorealistic rendering and advanced visualization
- **ROS-TCP-Connector** bridges ROS 2 and Unity seamlessly
- **URDF Importer** converts robot descriptions to Unity GameObjects
- **ArticulationBody** provides accurate joint simulation
- **VR integration** enables immersive teleoperation
- Unity complements Gazebo for complete digital twin workflows

In the next chapter, we'll explore **sensor simulation** in depth, covering cameras, LiDAR, IMUs, and sensor fusion techniques.
