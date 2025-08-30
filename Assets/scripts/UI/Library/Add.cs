using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class Add : MonoBehaviour
{
    [Header("Robot Prefabs")]
    [SerializeField] private GameObject[] robotPrefabs; // Array of robot prefabs to choose from
    
    [Header("Spawn Settings")]
    [SerializeField] private Transform spawnPoint; // Where to spawn the robot
    
    [Header("UI Buttons")]
    [SerializeField] private Button[] robotButtons; // UI buttons for each robot type
    
    [Header("Robot Management")]
    [SerializeField] private Transform robotParent; // Parent transform for spawned robots
    
    [Header("Robot Tracking")]
    public List<GameObject> spawnedRobots = new List<GameObject>(); // Public list of all spawned robots
    
    private GameObject currentRobot; // Reference to the currently spawned robot
    private Dictionary<Button, GameObject> buttonToPrefabMap; // Maps buttons to their corresponding prefabs
    
    void Start()
    {
        InitializeButtonMapping();
        SetupButtonListeners();
    }
    
    /// <summary>
    /// Initialize the mapping between buttons and prefabs
    /// </summary>
    private void InitializeButtonMapping()
    {
        buttonToPrefabMap = new Dictionary<Button, GameObject>();
        
        // Map each button to its corresponding prefab
        for (int i = 0; i < robotButtons.Length && i < robotPrefabs.Length; i++)
        {
            if (robotButtons[i] != null && robotPrefabs[i] != null)
            {
                buttonToPrefabMap.Add(robotButtons[i], robotPrefabs[i]);
            }
        }
    }
    
    /// <summary>
    /// Setup click listeners for all robot buttons
    /// </summary>
    private void SetupButtonListeners()
    {
        foreach (var kvp in buttonToPrefabMap)
        {
            Button button = kvp.Key;
            GameObject prefab = kvp.Value;
            
            button.onClick.AddListener(() => SpawnRobot(prefab));
        }
    }
    
    /// <summary>
    /// Spawn a robot prefab at the specified location
    /// </summary>
    /// <param name="robotPrefab">The robot prefab to spawn</param>
    public void SpawnRobot(GameObject robotPrefab)
    {
        if (robotPrefab == null)
        {
            Debug.LogWarning("Robot prefab is null!");
            return;
        }
        
        // Determine spawn position
        Vector3 spawnPosition = spawnPoint != null ? spawnPoint.position : Vector3.zero;
        
        // Spawn the robot
        GameObject spawnedRobot = Instantiate(robotPrefab, spawnPosition, Quaternion.identity);
        
        // Set parent if specified
        if (robotParent != null)
        {
            spawnedRobot.transform.SetParent(robotParent);
        }
        
        // Store reference to current robot
        currentRobot = spawnedRobot;
        
        // Add to the list of spawned robots
        spawnedRobots.Add(spawnedRobot);
        
        Debug.Log($"Spawned robot: {robotPrefab.name} at position: {spawnPosition}");
    }
    

    
    /// <summary>
    /// Spawn robot by index (useful for direct calls)
    /// </summary>
    /// <param name="robotIndex">Index of the robot prefab in the array</param>
    public void SpawnRobotByIndex(int robotIndex)
    {
        if (robotIndex >= 0 && robotIndex < robotPrefabs.Length)
        {
            SpawnRobot(robotPrefabs[robotIndex]);
        }
        else
        {
            Debug.LogError($"Invalid robot index: {robotIndex}. Valid range: 0-{robotPrefabs.Length - 1}");
        }
    }
    
    /// <summary>
    /// Destroy the currently spawned robot
    /// </summary>
    public void DestroyCurrentRobot()
    {
        if (currentRobot != null)
        {
            // Remove from the list before destroying
            spawnedRobots.Remove(currentRobot);
            DestroyImmediate(currentRobot);
            currentRobot = null;
            Debug.Log("Current robot destroyed");
        }
    }
    
    /// <summary>
    /// Get the currently spawned robot
    /// </summary>
    /// <returns>Reference to the current robot, or null if none exists</returns>
    public GameObject GetCurrentRobot()
    {
        return currentRobot;
    }
    
    /// <summary>
    /// Check if a robot is currently spawned
    /// </summary>
    /// <returns>True if a robot is currently spawned</returns>
    public bool HasRobotSpawned()
    {
        return currentRobot != null;
    }
    
    /// <summary>
    /// Set the spawn point transform
    /// </summary>
    /// <param name="newSpawnPoint">New spawn point transform</param>
    public void SetSpawnPoint(Transform newSpawnPoint)
    {
        spawnPoint = newSpawnPoint;
    }
    
    public void MoveCurrentRobot(Vector3 worldPosition, Quaternion worldRotation)
    {
        if (currentRobot == null)
        {
            return;
        }

        ArticulationBody rootBody = GetRootArticulationBody(currentRobot);
        if (rootBody != null)
        {
            rootBody.TeleportRoot(worldPosition, worldRotation);
        }
    }

    private ArticulationBody GetRootArticulationBody(GameObject go)
    {
        ArticulationBody[] bodies = go.GetComponentsInChildren<ArticulationBody>();
        for (int i = 0; i < bodies.Length; i++)
        {
            if (bodies[i].isRoot)
            {
                return bodies[i];
            }
        }
        return bodies.Length > 0 ? bodies[0] : null;
    }
}
