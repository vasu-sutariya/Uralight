using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;
using System.Reflection;

public class Selct : MonoBehaviour
{
    [Header("Robot Selection Panel")]
    [SerializeField] private GameObject robotButtonPrefab; // Prefab for robot selection buttons
    [SerializeField] private Transform verticalLayoutGroup; // Parent transform with VerticalLayoutGroup component
    
    [Header("Panel Management")]
    [SerializeField] private GameObject jogPanel; // JOG panel to enable when robot is selected
    [SerializeField] private GameObject jogListPanel; // Jog list panel to disable when robot is selected
    
    [Header("References")]
    [SerializeField] private Add robotManager;
    [SerializeField] private Jog jogController; // Reference to the Jog component
    
    private List<GameObject> currentRobots = new List<GameObject>();
    private List<GameObject> spawnedButtons = new List<GameObject>(); // Track spawned button instances
    
    void Start()
    {
        if (robotManager == null)
        {
            robotManager = FindFirstObjectByType<Add>();
        }
        
        if (jogController == null)
        {
            jogController = FindFirstObjectByType<Jog>();
        }
        
        
        InvokeRepeating("UpdateRobotButtons", 0.1f, 0.5f);
    }
    
    public void UpdateRobotButtons()
    {
        if (robotManager == null || robotButtonPrefab == null || verticalLayoutGroup == null) return;
        
        currentRobots = new List<GameObject>(robotManager.spawnedRobots);
        int robotCount = currentRobots.Count;
        
        //Debug.Log($"Found {robotCount} robots in the scene");
        
        // Clear existing buttons
        ClearExistingButtons();
        
        // Spawn new buttons for each robot
        for (int i = 0; i < robotCount; i++)
        {
            if (i < currentRobots.Count)
            {
                CreateRobotButton(currentRobots[i], i);
            }
        }
    }
    
    private void ClearExistingButtons()
    {
        // Destroy all existing button instances
        foreach (GameObject button in spawnedButtons)
        {
            if (button != null)
            {
                DestroyImmediate(button);
            }
        }
        spawnedButtons.Clear();
    }
    
    private void CreateRobotButton(GameObject robot, int robotIndex)
    {
        if (robot == null || robotButtonPrefab == null || verticalLayoutGroup == null) return;
        
        // Instantiate the button prefab as a child of the vertical layout group
        GameObject buttonInstance = Instantiate(robotButtonPrefab, verticalLayoutGroup);
        spawnedButtons.Add(buttonInstance);
        
        // Get the Button component
        Button button = buttonInstance.GetComponent<Button>();
        if (button == null)
        {
            //Debug.LogError("Robot button prefab must have a Button component!");
            return;
        }
        
        // Set the button text
        TMP_Text buttonText = buttonInstance.GetComponentInChildren<TMP_Text>();
        if (buttonText != null)
        {
            string robotName = robot.name.Replace("(Clone)", "").Trim();
            buttonText.text = $" {robotName}";
        }
        else
        {
            //Debug.LogWarning("Robot button prefab must have a TMP_Text component in children!");
        }
        
        // Set up the button click listener
        button.onClick.RemoveAllListeners();
        button.onClick.AddListener(() => SelectRobot(robot, robotIndex));
    }
    
    private void SelectRobot(GameObject selectedRobot, int robotIndex)
    {
        if (selectedRobot == null) return;
        
        //Debug.Log($"Selected robot: {selectedRobot.name} at index {robotIndex}");
        
        // Toggle panels - enable JOG panel and disable jog list panel
        TogglePanels();
        
        // Find the robot components from the selected robot GameObject
        UnityEncoder encoder = selectedRobot.GetComponent<UnityEncoder>();
        URInverseKinematics inverseKinematics = selectedRobot.GetComponent<URInverseKinematics>();
        UnityJointController jointController = selectedRobot.GetComponent<UnityJointController>();
        
        // If components are not found on the main GameObject, search in children
        if (encoder == null)
            encoder = selectedRobot.GetComponentInChildren<UnityEncoder>();
        if (inverseKinematics == null)
            inverseKinematics = selectedRobot.GetComponentInChildren<URInverseKinematics>();
        if (jointController == null)
            jointController = selectedRobot.GetComponentInChildren<UnityJointController>();
        
        // Assign the components to the Jog controller
        if (jogController != null)
        {
            // Use reflection to set the private fields in Jog component
            var jogType = typeof(Jog);
            
            var encoderField = jogType.GetField("encoder", BindingFlags.NonPublic | BindingFlags.Instance);
            if (encoderField != null && encoder != null)
            {
                encoderField.SetValue(jogController, encoder);
                //Debug.Log($"Assigned encoder: {encoder.name}");
            }
            
            var inverseKinematicsField = jogType.GetField("inverseKinematics", BindingFlags.NonPublic | BindingFlags.Instance);
            if (inverseKinematicsField != null && inverseKinematics != null)
            {
                inverseKinematicsField.SetValue(jogController, inverseKinematics);
                //Debug.Log($"Assigned inverse kinematics: {inverseKinematics.name}");
            }
            
            var jointControllerField = jogType.GetField("jointController", BindingFlags.NonPublic | BindingFlags.Instance);
            if (jointControllerField != null && jointController != null)
            {
                jointControllerField.SetValue(jogController, jointController);
                //Debug.Log($"Assigned joint controller: {jointController.name}");
            }
            
            //Debug.Log($"Successfully assigned robot components to Jog controller for robot: {selectedRobot.name}");
        }
        else
        {
            //Debug.LogError("Jog controller not found! Please assign it in the inspector.");
        }
    }
    
    /// <summary>
    /// Toggle between JOG panel and jog list panel
    /// </summary>
    private void TogglePanels()
    {
        if (jogPanel != null)
        {
            jogPanel.SetActive(true);
        }
        
        if (jogListPanel != null)
        {
            jogListPanel.SetActive(false);
        }
    }
    
    /// <summary>
    /// Public method to return to the jog list panel (can be called from a back button)
    /// </summary>
    public void ReturnToJogList()
    {
        if (jogPanel != null)
        {
            jogPanel.SetActive(false);
        }
        
        if (jogListPanel != null)
        {
            jogListPanel.SetActive(true);
        }
    }
    
    public int GetRobotCount()
    {
        return currentRobots.Count;
    }
    
    /// <summary>
    /// Get the currently selected robot GameObject
    /// </summary>
    /// <param name="robotIndex">Index of the robot</param>
    /// <returns>GameObject of the robot, or null if not found</returns>
    public GameObject GetRobotGameObject(int robotIndex)
    {
        if (robotIndex >= 0 && robotIndex < currentRobots.Count)
        {
            return currentRobots[robotIndex];
        }
        return null;
    }
    
    void OnDestroy()
    {
        CancelInvoke("UpdateRobotButtons");
        ClearExistingButtons();
    }
}
