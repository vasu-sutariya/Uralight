using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;
using System.IO;
using System;

public class CreateUI : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private Button createUIButton; // Button to open the custom UI panel
    [SerializeField] private GameObject customUIPanel; // The main custom UI panel
    [SerializeField] private TMP_Dropdown commandFilesDropdown; // Dropdown for saved command files
    [SerializeField] private Toggle setToggle; // Toggle for Set mode
    [SerializeField] private Transform buttonContainer; // Container for dynamically created buttons
    [SerializeField] private GameObject buttonPrefab; // Prefab for the assignable buttons
    
    [Header("Other Panels to Hide")]
    [SerializeField] private GameObject[] panelsToHide; // Array of panels to hide when custom UI is open
    
    [Header("References")]
    [SerializeField] private SaveFile saveFileScript; // Reference to SaveFile script
    [SerializeField] private Play playScript; // Reference to Play script for executing commands
    [SerializeField] private UIManager uiManager; // Reference to UI Manager for panel management
    
    [Header("Settings")]
    [SerializeField] private string saveDirectory = "RobotPrograms";
    [SerializeField] private string fileExtension = ".json";
    
    private List<GameObject> createdButtons = new List<GameObject>();
    private List<CommandProgram> loadedPrograms = new List<CommandProgram>();
    private bool isSetMode = false;
    private bool isPanelOpen = false;
    
    private string SavePath => Path.Combine(Application.persistentDataPath, saveDirectory);
    
    void Awake()
    {
        // Find references if not assigned
        if (saveFileScript == null)
            saveFileScript = FindFirstObjectByType<SaveFile>();
        
        if (playScript == null)
            playScript = FindFirstObjectByType<Play>();
        
        if (uiManager == null)
            uiManager = FindFirstObjectByType<UIManager>();
        
        WireUpUI();
        LoadCommandFiles();
    }
    
    private void WireUpUI()
    {
        // Wire up create UI button
        if (createUIButton != null)
        {
            createUIButton.onClick.RemoveAllListeners();
            createUIButton.onClick.AddListener(ToggleCustomUIPanel);
        }
        
        // Wire up dropdown
        if (commandFilesDropdown != null)
        {
            commandFilesDropdown.onValueChanged.RemoveAllListeners();
            commandFilesDropdown.onValueChanged.AddListener(OnCommandFileSelected);
        }
        
        // Wire up set toggle
        if (setToggle != null)
        {
            setToggle.onValueChanged.RemoveAllListeners();
            setToggle.onValueChanged.AddListener(OnSetToggleChanged);
        }
        
        // Initially hide the custom UI panel
        if (customUIPanel != null)
            customUIPanel.SetActive(false);
    }
    
    public void ToggleCustomUIPanel()
    {
        isPanelOpen = !isPanelOpen;
        
        if (customUIPanel != null)
        {
            customUIPanel.SetActive(isPanelOpen);
        }
        
        if (isPanelOpen)
        {
            // Hide all other panels using UI Manager
            if (uiManager != null)
            {
                uiManager.OnCustomUIOpened();
            }
            else
            {
                // Fallback to manual panel hiding
                HideOtherPanels();
            }
            
            // Refresh the dropdown
            LoadCommandFiles();
        }
        else
        {
            // Notify UI Manager that custom UI is closed
            if (uiManager != null)
            {
                uiManager.OnCustomUIClosed();
            }
        }
    }
    
    private void HideOtherPanels()
    {
        if (panelsToHide != null)
        {
            foreach (GameObject panel in panelsToHide)
            {
                if (panel != null)
                    panel.SetActive(false);
            }
        }
    }
    
    private void LoadCommandFiles()
    {
        if (commandFilesDropdown == null) return;
        
        commandFilesDropdown.ClearOptions();
        loadedPrograms.Clear();
        
        try
        {
            if (!Directory.Exists(SavePath)) return;
            
            string[] files = Directory.GetFiles(SavePath, "*" + fileExtension);
            List<string> programNames = new List<string>();
            
            foreach (string file in files)
            {
                try
                {
                    string json = File.ReadAllText(file);
                    CommandProgram program = JsonUtility.FromJson<CommandProgram>(json);
                    if (program != null)
                    {
                        programNames.Add(program.programName);
                        loadedPrograms.Add(program);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"Failed to read program file {file}: {e.Message}");
                }
            }
            
            commandFilesDropdown.AddOptions(programNames);
            Debug.Log($"Loaded {programNames.Count} command files into dropdown");
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to load command files: {e.Message}");
        }
    }
    
    private void OnCommandFileSelected(int index)
    {
        Debug.Log($"Dropdown selection changed to index: {index}");
        
        if (index < 0 || index >= loadedPrograms.Count) 
        {
            Debug.LogError($"Invalid dropdown index: {index}. Loaded programs count: {loadedPrograms.Count}");
            return;
        }
        
        CommandProgram selectedProgram = loadedPrograms[index];
        Debug.Log($"Selected program: {selectedProgram.programName}");
        CreateButtonForProgram(selectedProgram);
    }
    
    private void CreateButtonForProgram(CommandProgram program)
    {
        if (buttonPrefab == null) 
        {
            Debug.LogError("Button prefab is not assigned!");
            return;
        }
        
        if (buttonContainer == null)
        {
            Debug.LogError("Button container is not assigned!");
            return;
        }
        
        // Debug button container info
        Debug.Log($"Button container: {buttonContainer.name}, Active: {buttonContainer.gameObject.activeInHierarchy}");
        RectTransform containerRect = buttonContainer.GetComponent<RectTransform>();
        if (containerRect != null)
        {
            Debug.Log($"Container size: {containerRect.sizeDelta}, Position: {containerRect.anchoredPosition}");
        }
        
        // Create new button instance
        GameObject newButton = Instantiate(buttonPrefab, buttonContainer);
        createdButtons.Add(newButton);
        
        // Ensure the button is properly positioned and visible
        RectTransform buttonRect = newButton.GetComponent<RectTransform>();
        if (buttonRect != null)
        {
            // Reset position and scale
            buttonRect.anchoredPosition = Vector2.zero;
            buttonRect.localScale = Vector3.one;
            
            // Make sure it's active
            newButton.SetActive(true);
            
            Debug.Log($"Button created and positioned. Active: {newButton.activeInHierarchy}, Position: {buttonRect.anchoredPosition}");
        }
        else
        {
            Debug.LogError("Created button does not have a RectTransform component!");
        }
        
        Debug.Log($"Created button for program: {program.programName}");
        
        // Force layout refresh if container has layout components
        LayoutGroup layoutGroup = buttonContainer.GetComponent<LayoutGroup>();
        if (layoutGroup != null)
        {
            LayoutRebuilder.ForceRebuildLayoutImmediate(containerRect);
            Debug.Log("Forced layout rebuild for container");
        }
        
        // Set button text to program name
        TMP_Text buttonText = newButton.GetComponentInChildren<TMP_Text>();
        if (buttonText != null)
        {
            buttonText.text = program.programName;
        }
        
        // Get the button component
        Button buttonComponent = newButton.GetComponent<Button>();
        if (buttonComponent != null)
        {
            // Remove any existing listeners
            buttonComponent.onClick.RemoveAllListeners();
            
            if (isSetMode)
            {
                // In Set mode: assign button to play the program
                buttonComponent.onClick.AddListener(() => PlayProgram(program));
                
                // Disable drag functionality
                DragHandler dragHandler = newButton.GetComponent<DragHandler>();
                if (dragHandler != null)
                {
                    dragHandler.enabled = false;
                }
            }
            else
            {
                // In non-Set mode: enable drag functionality
                DragHandler dragHandler = newButton.GetComponent<DragHandler>();
                if (dragHandler != null)
                {
                    dragHandler.enabled = true;
                }
                
                // Remove any program assignment
                buttonComponent.onClick.RemoveAllListeners();
            }
        }
    }
    
    private void OnSetToggleChanged(bool isOn)
    {
        isSetMode = isOn;
        UpdateAllButtons();
    }
    
    private void UpdateAllButtons()
    {
        for (int i = 0; i < createdButtons.Count; i++)
        {
            GameObject button = createdButtons[i];
            if (button == null) continue;
            
            Button buttonComponent = button.GetComponent<Button>();
            DragHandler dragHandler = button.GetComponent<DragHandler>();
            
            if (isSetMode)
            {
                // Set mode: disable drag, enable program assignment
                if (dragHandler != null)
                    dragHandler.enabled = false;
                
                // Find the corresponding program for this button
                TMP_Text buttonText = button.GetComponentInChildren<TMP_Text>();
                if (buttonText != null)
                {
                    CommandProgram program = loadedPrograms.Find(p => p.programName == buttonText.text);
                    if (program != null)
                    {
                        buttonComponent.onClick.RemoveAllListeners();
                        buttonComponent.onClick.AddListener(() => PlayProgram(program));
                    }
                }
            }
            else
            {
                // Non-Set mode: enable drag, disable program assignment
                if (dragHandler != null)
                    dragHandler.enabled = true;
                
                buttonComponent.onClick.RemoveAllListeners();
            }
        }
    }
    
    private void PlayProgram(CommandProgram program)
    {
        if (playScript == null)
        {
            Debug.LogError("Play script not found!");
            return;
        }
        
        // Load the program into the command manager and play it
        if (saveFileScript != null)
        {
            // Use the SaveFile script to load the program
            saveFileScript.LoadProgramByName(program.programName);
        }
        
        // Execute the loaded program
        playScript.OnPlayButtonPressed();
    }
    
    // Public method to refresh the dropdown (can be called externally)
    public void RefreshCommandFiles()
    {
        LoadCommandFiles();
    }
    
    // Public method to clear all created buttons
    public void ClearAllButtons()
    {
        foreach (GameObject button in createdButtons)
        {
            if (button != null)
                DestroyImmediate(button);
        }
        createdButtons.Clear();
    }
    
    // Public method to close the panel
    public void ClosePanel()
    {
        if (customUIPanel != null)
        {
            customUIPanel.SetActive(false);
            isPanelOpen = false;
        }
    }
    
    // Public method to test button creation (for debugging)
    public void TestCreateButton()
    {
        if (loadedPrograms.Count > 0)
        {
            CreateButtonForProgram(loadedPrograms[0]);
        }
        else
        {
            Debug.LogWarning("No programs loaded to test with");
        }
    }
}
