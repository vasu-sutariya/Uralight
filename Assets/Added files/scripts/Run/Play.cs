using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

public class Play : MonoBehaviour
{
    [Header("Play Button")]
    [SerializeField] private Button playButton;
    [SerializeField] private Button stopButton; 
    
    [Header("Providers")]
    [SerializeField] private AddCommand commandManager;
    [SerializeField] private Add robotManager;
    
    [Header("Execution Settings")]
    [SerializeField] private float commandExecutionDelay = 0.1f; // Delay between commands
    
    private bool isExecuting = false;
    private Coroutine executionCoroutine;
    
    void Awake()
    {
        if (commandManager == null)
        {
            commandManager = FindFirstObjectByType<AddCommand>();
        }
        
        if (robotManager == null)
        {
            robotManager = FindFirstObjectByType<Add>();
        }
        
        WireUpPlayButton();
    }
    
    private void WireUpPlayButton()
    {
        if (playButton != null)
        {
            
            playButton.onClick.AddListener(OnPlayButtonPressed);
        }
        if (stopButton != null)
        {
            
            stopButton.onClick.AddListener(StopExecution);
        }
    }
    
    public void OnPlayButtonPressed()
    {
        if (isExecuting)
        {
            //Debug.Log("Already executing commands. Please wait for completion.");
            return;
        }
        
        if (commandManager == null)
        {
            //Debug.LogError("Command manager not found!");
            return;
        }
        
        var commands = commandManager.GetCommands();
        if (commands == null || commands.Count == 0)
        {
            //Debug.Log("No commands to execute!");
            return;
        }
        
        //Debug.Log($"Starting execution of {commands.Count} commands...");
        isExecuting = true;
        executionCoroutine = StartCoroutine(ExecuteCommands(commands));
    }
    
    private IEnumerator ExecuteCommands(IReadOnlyList<AddCommand.SimpleCommand> commands)
    {
        for (int i = 0; i < commands.Count; i++)
        {
            var command = commands[i];
            
            if (command == null)
            {
                //Debug.LogWarning($"Command at index {i} is null, skipping...");
                continue;
            }
            
            //Debug.Log($"Executing command {i + 1}/{commands.Count}: {command.commandType}");
            
            switch (command.commandType)
            {
                case "MOVEJ":
                    yield return ExecuteMoveJCommand(command);
                    break;
                    
                case "WAIT":
                    yield return ExecuteWaitCommand(command);
                    break;
                    
                case "MOVEL":
                    //Debug.Log($"Skipping MOVEL command (not implemented yet)");
                    break;
                    
                case "SET":
                    //Debug.Log($"Skipping SET command (not implemented yet)");
                    break;

                case "ATTACH":
                    yield return ExecuteAttachCommand(command);
                    break;

                case "DETACH":
                    yield return ExecuteDetachCommand(command);
                    break;

                default:
                    //Debug.LogWarning($"Unknown command type: {command.commandType}");
                    break;
            }
            
            // Small delay between commands
            if (i < commands.Count - 1) // Don't delay after the last command
            {
                yield return new WaitForSeconds(commandExecutionDelay);
            }
        }
        
        //Debug.Log("Command execution completed!");
        isExecuting = false;
        executionCoroutine = null;
    }
    
    private IEnumerator ExecuteMoveJCommand(AddCommand.SimpleCommand command)
    {
        if (!command.isSet)
        {
            //Debug.LogWarning("MOVEJ command is not set, skipping...");
            yield break;
        }
        
        GameObject robot = GetRobotByName(command.robotName);
        if (robot == null)
        {
            //Debug.LogError($"Robot '{command.robotName}' not found!");
            yield break;
        }
        
        //Debug.Log($"Executing MOVEJ to angles: [{string.Join(", ", command.angles)}] with blend={command.blendRadius}, time={command.time}");
        
        // Execute the movement with blend radius and time
        MoveRobotToAngles(robot, command.angles, command.velocity, command.acceleration, command.blendRadius, command.time);
        
        // Wait for movement to complete
        yield return StartCoroutine(WaitForMovementComplete(robot));
    }
    
    private IEnumerator ExecuteAttachCommand(AddCommand.SimpleCommand command)
    {
        if (!command.isSet || string.IsNullOrEmpty(command.attachedObjectName))
        {
            Debug.LogWarning("ATTACH command is not set or missing object name, skipping...");
            yield break;
        }
        
        GameObject robot = GetRobotByName(command.robotName);
        if (robot == null)
        {
            Debug.LogError($"Robot '{command.robotName}' not found for ATTACH command!");
            yield break;
        }
        
        // Find the object to attach
        GameObject objectToAttach = GameObject.Find(command.attachedObjectName);
        if (objectToAttach == null)
        {
            Debug.LogError($"Object '{command.attachedObjectName}' not found for ATTACH command!");
            yield break;
        }
        
        // Find the attachment point (wrist3 for robots, or the robot itself for CAD imports)
        Transform targetParent = robot.transform;
        if (IsRobot(robot))
        {
            Transform wrist3 = FindRobotWrist3(robot);
            if (wrist3 != null)
            {
                targetParent = wrist3;
            }
        }
        
        // Attach the object to the target parent
        objectToAttach.transform.SetParent(targetParent);
        
        Debug.Log($"Executed ATTACH: {objectToAttach.name} attached to {targetParent.name}");
        
        // Small delay to allow the attachment to complete
        yield return new WaitForSeconds(0.1f);
    }
    
    private IEnumerator ExecuteDetachCommand(AddCommand.SimpleCommand command)
    {
        if (!command.isSet || string.IsNullOrEmpty(command.attachedObjectName))
        {
            //Debug.LogWarning("DETACH command is not set or missing object name, skipping...");
            yield break;
        }
        
        // Find the object to detach
        GameObject objectToDetach = GameObject.Find(command.attachedObjectName);
        if (objectToDetach == null)
        {
            //Debug.LogError($"Object '{command.attachedObjectName}' not found for DETACH command!");
            yield break;
        }
        
        // Check if the object has a parent
        if (objectToDetach.transform.parent != null)
        {
            // Store the current world position and rotation
            Vector3 worldPosition = objectToDetach.transform.position;
            Quaternion worldRotation = objectToDetach.transform.rotation;
            
            // Detach from parent
            objectToDetach.transform.SetParent(null);
            
            // Maintain the same world position and rotation
            objectToDetach.transform.position = worldPosition;
            objectToDetach.transform.rotation = worldRotation;
            
            //Debug.Log($"Executed DETACH: {objectToDetach.name} detached from parent");
        }
        else
        {
            //Debug.LogWarning($"{objectToDetach.name} is not attached to any parent for DETACH command");
        }
        
        // Small delay to allow the detachment to complete
        yield return new WaitForSeconds(0.1f);
    }
    
    private IEnumerator ExecuteWaitCommand(AddCommand.SimpleCommand command)
    {
        float waitTime = command.time;
        //Debug.Log($"Executing WAIT for {waitTime} seconds");
        
        yield return new WaitForSeconds(waitTime);
        
        //Debug.Log("WAIT command completed");
    }
    
    private IEnumerator WaitForMovementComplete(GameObject robot)
    {
        UnityTrajControl trajControl = robot.GetComponentInChildren<UnityTrajControl>();
        
        if (trajControl != null)
        {
            // Wait for trajectory to complete
            while (trajControl.isTrajectoryActive)
            {
                yield return null;
            }
        }
        else
        {
            // Fallback: wait a fixed time if no trajectory control
            yield return new WaitForSeconds(2.0f);
        }
        
        //Debug.Log("MOVEJ command completed");
    }
    
    private GameObject GetRobotByName(string robotName)
    {
        if (robotManager == null) return null;
        
        // First try to get the current robot
        GameObject currentRobot = robotManager.GetCurrentRobot();
        if (currentRobot != null && SanitizeName(currentRobot.name) == robotName)
        {
            return currentRobot;
        }
        
        // If not found, search through all spawned robots
        var spawnedRobots = robotManager.spawnedRobots;
        foreach (var robot in spawnedRobots)
        {
            if (robot != null && SanitizeName(robot.name) == robotName)
            {
                return robot;
            }
        }
        
        return null;
    }
    
    private void MoveRobotToAngles(GameObject robot, float[] targetAngles, float velocity, float acceleration, float blendRadius = -1f, float time = -1f)
    {
        if (robot == null || targetAngles == null || targetAngles.Length < 6) return;
        
        UnityTrajControl traj = robot.GetComponentInChildren<UnityTrajControl>();
        if (traj != null)
        {
            traj.MoveJ(targetAngles, velocity, acceleration, blendRadius, time);
            return;
        }
        
        UnityJointController jointController = robot.GetComponentInChildren<UnityJointController>();
        if (jointController != null)
        {
            jointController.ChangeUnityTargetAngles(targetAngles);
        }
    }
    
    private static string SanitizeName(string raw)
    {
        if (string.IsNullOrEmpty(raw)) return string.Empty;
        return raw.Replace("(Clone)", string.Empty).Trim();
    }
    
    public void StopExecution()
    {
        if (executionCoroutine != null)
        {
            StopCoroutine(executionCoroutine);
            executionCoroutine = null;
        }
        isExecuting = false;
        
        // Stop all trajectory commands on all robots
        StopAllTrajectories();
        
        //Debug.Log("Command execution stopped by user");
    }
    
    private void StopAllTrajectories()
    {
        if (robotManager == null) return;
        
        // Stop trajectory on current robot
        GameObject currentRobot = robotManager.GetCurrentRobot();
        if (currentRobot != null)
        {
            StopRobotTrajectory(currentRobot);
        }
        
        // Stop trajectory on all spawned robots
        var spawnedRobots = robotManager.spawnedRobots;
        foreach (var robot in spawnedRobots)
        {
            if (robot != null)
            {
                StopRobotTrajectory(robot);
            }
        }
    }
    
    private void StopRobotTrajectory(GameObject robot)
    {
        if (robot == null) return;
        
        UnityTrajControl trajControl = robot.GetComponentInChildren<UnityTrajControl>();
        if (trajControl != null)
        {
            trajControl.isTrajectoryActive = false;
            //Debug.Log($"Stopped trajectory on robot: {robot.name}");
        }
        
        // Also stop any ongoing joint controller movements by setting target to current position
        UnityJointController jointController = robot.GetComponentInChildren<UnityJointController>();
        UnityEncoder encoder = robot.GetComponentInChildren<UnityEncoder>();
        if (jointController != null && encoder != null)
        {
            float[] currentAngles = encoder.GetUnityAngles();
            jointController.ChangeUnityTargetAngles(currentAngles);
            //Debug.Log($"Stopped joint movement on robot: {robot.name}");
        }
    }
    
    public bool IsExecuting()
    {
        return isExecuting;
    }
    
    /// <summary>
    /// Checks if a GameObject is a robot by looking for robot-specific components
    /// </summary>
    private bool IsRobot(GameObject go)
    {
        if (go == null) return false;
        if (go.GetComponentInChildren<UnityJointController>() != null) return true;
        if (go.GetComponentInChildren<ArticulationBody>() != null) return true;
        return false;
    }
    
    /// <summary>
    /// Finds the robot's wrist3 (last child) to use as the end effector attachment point
    /// </summary>
    private Transform FindRobotWrist3(GameObject robot)
    {
        if (robot == null) return null;

        // Get all children recursively
        Transform[] allChildren = robot.GetComponentsInChildren<Transform>();
        
        // Find the deepest child (highest depth)
        Transform wrist3 = null;
        int maxDepth = -1;
        
        foreach (Transform child in allChildren)
        {
            if (child == robot.transform) continue; // Skip the robot root
            
            int depth = GetTransformDepth(child, robot.transform);
            if (depth > maxDepth)
            {
                maxDepth = depth;
                wrist3 = child;
            }
        }
        
        return wrist3;
    }
    
    /// <summary>
    /// Calculates the depth of a transform relative to a root transform
    /// </summary>
    private int GetTransformDepth(Transform child, Transform root)
    {
        int depth = 0;
        Transform current = child;
        
        while (current != null && current != root)
        {
            depth++;
            current = current.parent;
        }
        
        return depth;
    }
}
