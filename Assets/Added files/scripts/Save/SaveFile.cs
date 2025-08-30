using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;
using System.IO;
using System;

[System.Serializable]
public class CommandProgram
{
    public string programName;
    public string createdDate;
    public List<AddCommand.SimpleCommand> commands;
    
    public CommandProgram()
    {
        programName = "New Program";
        createdDate = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        commands = new List<AddCommand.SimpleCommand>();
    }
}

public class SaveFile : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private Button saveButton;
    [SerializeField] private Button loadButton;
    [SerializeField] private TMP_InputField programNameInput;
    [SerializeField] private TMP_Dropdown loadProgramDropdown;
    
    [Header("References")]
    [SerializeField] private AddCommand addCommandScript;
    
    [Header("File Settings")]
    [SerializeField] private string saveDirectory = "RobotPrograms";
    [SerializeField] private string fileExtension = ".json";
    
    private string SavePath => Path.Combine(Application.persistentDataPath, saveDirectory);
    
    void Awake()
    {
        if (addCommandScript == null)
        {
            addCommandScript = FindFirstObjectByType<AddCommand>();
        }
        
        WireUpButtons();
        CreateSaveDirectory();
        RefreshLoadDropdown();
    }
    
    private void WireUpButtons()
    {
        if (saveButton != null)
        {
            saveButton.onClick.RemoveAllListeners();
            saveButton.onClick.AddListener(SaveProgram);
        }
        
        if (loadButton != null)
        {
            loadButton.onClick.RemoveAllListeners();
            loadButton.onClick.AddListener(LoadSelectedProgram);
        }
        
        if (loadProgramDropdown != null)
        {
            loadProgramDropdown.onValueChanged.RemoveAllListeners();
            loadProgramDropdown.onValueChanged.AddListener(OnLoadDropdownChanged);
        }
    }
    
    public void SaveProgram()
    {
        if (addCommandScript == null)
        {
            Debug.LogError("AddCommand script not found!");
            return;
        }
        
        // Get program name
        string programName = GetProgramName();
        if (string.IsNullOrEmpty(programName))
        {
            Debug.LogWarning("Please enter a program name!");
            return;
        }
        
        // Create program data
        CommandProgram program = new CommandProgram();
        program.programName = programName;
        program.commands = new List<AddCommand.SimpleCommand>(addCommandScript.GetCommands());
        
        // Convert to JSON
        string json = JsonUtility.ToJson(program, true);
        
        // Save to file
        string fileName = SanitizeFileName(programName) + fileExtension;
        string filePath = Path.Combine(SavePath, fileName);
        
        try
        {
            File.WriteAllText(filePath, json);
            Debug.Log($"Program saved successfully: {filePath}");
            
            // Refresh the load dropdown
            RefreshLoadDropdown();
            
            // Show success message
            ShowSaveSuccessMessage(programName);
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to save program: {e.Message}");
            ShowSaveErrorMessage(e.Message);
        }
    }
    
    public void LoadSelectedProgram()
    {
        if (loadProgramDropdown == null || addCommandScript == null) return;
        
        int selectedIndex = loadProgramDropdown.value;
        if (selectedIndex < 0 || selectedIndex >= loadProgramDropdown.options.Count) return;
        
        string selectedFileName = loadProgramDropdown.options[selectedIndex].text;
        if (string.IsNullOrEmpty(selectedFileName)) return;
        
        string filePath = Path.Combine(SavePath, selectedFileName + fileExtension);
        
        try
        {
            if (!File.Exists(filePath))
            {
                Debug.LogError($"Program file not found: {filePath}");
                return;
            }
            
            string json = File.ReadAllText(filePath);
            CommandProgram program = JsonUtility.FromJson<CommandProgram>(json);
            
            if (program == null)
            {
                Debug.LogError("Failed to parse program file!");
                return;
            }
            
            // Clear current commands in AddCommand
            ClearCurrentCommands();
            
            // Load the commands
            LoadCommandsToAddCommand(program.commands);
            
            // Update UI fields
            if (programNameInput != null) programNameInput.text = program.programName;
            
            Debug.Log($"Program loaded successfully: {program.programName}");
            ShowLoadSuccessMessage(program.programName);
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to load program: {e.Message}");
            ShowLoadErrorMessage(e.Message);
        }
    }
    
    public void DeleteSelectedProgram()
    {
        if (loadProgramDropdown == null) return;
        
        int selectedIndex = loadProgramDropdown.value;
        if (selectedIndex < 0 || selectedIndex >= loadProgramDropdown.options.Count) return;
        
        string selectedFileName = loadProgramDropdown.options[selectedIndex].text;
        if (string.IsNullOrEmpty(selectedFileName)) return;
        
        string filePath = Path.Combine(SavePath, selectedFileName + fileExtension);
        
        try
        {
            if (File.Exists(filePath))
            {
                File.Delete(filePath);
                Debug.Log($"Program deleted: {selectedFileName}");
                RefreshLoadDropdown();
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to delete program: {e.Message}");
        }
    }
    
    private void ClearCurrentCommands()
    {
        if (addCommandScript != null)
        {
            addCommandScript.ClearAllCommands();
        }
    }
    
    private void LoadCommandsToAddCommand(List<AddCommand.SimpleCommand> commands)
    {
        if (addCommandScript == null || commands == null) return;
        
        // Clear existing commands first
        ClearCurrentCommands();
        
        // Add each command back
        foreach (var command in commands)
        {
            addCommandScript.AddCommandProgrammatically(command);
        }
    }
    
    private void RefreshLoadDropdown()
    {
        if (loadProgramDropdown == null) return;
        
        loadProgramDropdown.ClearOptions();
        
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
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"Failed to read program file {file}: {e.Message}");
                }
            }
            
            loadProgramDropdown.AddOptions(programNames);
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to refresh load dropdown: {e.Message}");
        }
    }
    
    private void OnLoadDropdownChanged(int index)
    {
        // Optional: Preview program info when selected
        if (index >= 0 && index < loadProgramDropdown.options.Count)
        {
            string selectedName = loadProgramDropdown.options[index].text;
            // Could show program description or other info here
        }
    }
    
    private void CreateSaveDirectory()
    {
        try
        {
            if (!Directory.Exists(SavePath))
            {
                Directory.CreateDirectory(SavePath);
                Debug.Log($"Created save directory: {SavePath}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to create save directory: {e.Message}");
        }
    }
    
    private string GetProgramName()
    {
        return programNameInput != null ? programNameInput.text.Trim() : "New Program";
    }
    

    
    private string SanitizeFileName(string fileName)
    {
        // Remove invalid characters for file names
        char[] invalidChars = Path.GetInvalidFileNameChars();
        foreach (char c in invalidChars)
        {
            fileName = fileName.Replace(c, '_');
        }
        return fileName;
    }
    
    private void ShowSaveSuccessMessage(string programName)
    {
        // You can implement UI feedback here
        Debug.Log($"✓ Program '{programName}' saved successfully!");
    }
    
    private void ShowSaveErrorMessage(string error)
    {
        // You can implement UI feedback here
        Debug.LogError($"✗ Save failed: {error}");
    }
    
    private void ShowLoadSuccessMessage(string programName)
    {
        // You can implement UI feedback here
        Debug.Log($"✓ Program '{programName}' loaded successfully!");
    }
    
    private void ShowLoadErrorMessage(string error)
    {
        // You can implement UI feedback here
        Debug.LogError($"✗ Load failed: {error}");
    }
    
    // Public methods for external access
    public void RefreshProgramsList()
    {
        RefreshLoadDropdown();
    }
    
    public string GetSaveDirectory()
    {
        return SavePath;
    }
    
    public List<string> GetAvailablePrograms()
    {
        List<string> programs = new List<string>();
        
        try
        {
            if (!Directory.Exists(SavePath)) return programs;
            
            string[] files = Directory.GetFiles(SavePath, "*" + fileExtension);
            foreach (string file in files)
            {
                try
                {
                    string json = File.ReadAllText(file);
                    CommandProgram program = JsonUtility.FromJson<CommandProgram>(json);
                    if (program != null)
                    {
                        programs.Add(program.programName);
                    }
                }
                catch
                {
                    // Skip corrupted files
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to get available programs: {e.Message}");
        }
        
        return programs;
    }
    
    // Debug method to get current program as JSON string
    public string GetCurrentProgramAsJson()
    {
        if (addCommandScript == null) return "{}";
        
        CommandProgram program = new CommandProgram();
        program.programName = GetProgramName();
        program.commands = new List<AddCommand.SimpleCommand>(addCommandScript.GetCommands());
        
        return JsonUtility.ToJson(program, true);
    }
    
    // Method to save program with custom name (for external calls)
    public void SaveProgramWithName(string programName)
    {
        if (addCommandScript == null)
        {
            Debug.LogError("AddCommand script not found!");
            return;
        }
        
        if (string.IsNullOrEmpty(programName))
        {
            Debug.LogWarning("Program name cannot be empty!");
            return;
        }
        
        // Create program data
        CommandProgram program = new CommandProgram();
        program.programName = programName;
        program.commands = new List<AddCommand.SimpleCommand>(addCommandScript.GetCommands());
        
        // Convert to JSON
        string json = JsonUtility.ToJson(program, true);
        
        // Save to file
        string fileName = SanitizeFileName(programName) + fileExtension;
        string filePath = Path.Combine(SavePath, fileName);
        
        try
        {
            File.WriteAllText(filePath, json);
            Debug.Log($"Program saved successfully: {filePath}");
            
            // Refresh the load dropdown
            RefreshLoadDropdown();
            
            // Show success message
            ShowSaveSuccessMessage(programName);
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to save program: {e.Message}");
            ShowSaveErrorMessage(e.Message);
        }
    }
}
