using UnityEngine;
using UnityEngine.UI;
using System;
using System.IO;
using System.Runtime.InteropServices;

public class Add3DModel : MonoBehaviour
{
    [Header("3D Model Settings")]
    [Tooltip("Selected OBJ file path from file explorer")]
    public string selectedFilePath = "";
    
    [Tooltip("Last selected file name for display")]
    public string selectedFileName = "No file selected";
    
    [Header("UI References")]
    [Tooltip("Button to open file explorer and select OBJ file")]
    public Button selectFileButton;
    
    [Header("Spawn Settings")]
    [Tooltip("Position where the model will be spawned")]
    public Vector3 spawnPosition = Vector3.zero;
    
    [Tooltip("Scale of the spawned model")]
    public Vector3 spawnScale = Vector3.one;
    
    [Tooltip("Rotation of the spawned model")]
    public Vector3 spawnRotation = Vector3.zero;
    
    private void Start()
    {
        // If no select file button is assigned, try to find one in children
        if (selectFileButton == null)
        {
            selectFileButton = GetComponentInChildren<Button>();
        }
        
        // Add listener to the select file button
        if (selectFileButton != null)
        {
            selectFileButton.onClick.AddListener(OpenFileExplorer);
        }
        else
        {
            Debug.LogWarning("No select file button found! Please assign a button or add one as a child.");
        }
    }
    
    /// <summary>
    /// Opens Windows file explorer to select an OBJ file
    /// </summary>
    public void OpenFileExplorer()
    {
        string path = OpenFileDialog("Select OBJ File", "obj");
        if (!string.IsNullOrEmpty(path))
        {
            selectedFilePath = path;
            selectedFileName = Path.GetFileName(path);
            Debug.Log($"Selected file: {selectedFileName}");
            
            // Automatically spawn the model when file is selected
            Spawn3DModel();
        }
    }
    
    /// <summary>
    /// Spawns the 3D model at the specified position
    /// </summary>
    public void Spawn3DModel()
    {
        if (string.IsNullOrEmpty(selectedFilePath))
        {
            Debug.LogError("Please select an OBJ file first!");
            return;
        }
        
        if (!File.Exists(selectedFilePath))
        {
            Debug.LogError($"File does not exist: {selectedFilePath}");
            return;
        }
        
        // Load and spawn the OBJ file
        GameObject objModel = LoadOBJFromFile(selectedFilePath);
        
        if (objModel != null)
        {
            // Instantiate the model at the spawn position
            GameObject spawnedModel = Instantiate(objModel, spawnPosition, Quaternion.Euler(spawnRotation));
            spawnedModel.transform.localScale = spawnScale;
            
            // Set a default name
            spawnedModel.name = selectedFileName + "_Instance";
            
            Debug.Log($"Successfully spawned {selectedFileName} at position {spawnPosition}");
        }
        else
        {
            Debug.LogError($"Failed to load OBJ file: {selectedFilePath}");
        }
    }
    
    /// <summary>
    /// Loads an OBJ file from a file path
    /// </summary>
    /// <param name="filePath">Full path to the OBJ file</param>
    /// <returns>GameObject loaded from the OBJ file</returns>
    private GameObject LoadOBJFromFile(string filePath)
    {
        try
        {
            // Import the OBJ file using Unity's AssetDatabase (Editor only)
            #if UNITY_EDITOR
            string relativePath = GetRelativePath(filePath);
            if (!string.IsNullOrEmpty(relativePath))
            {
                // Import the asset
                UnityEditor.AssetDatabase.ImportAsset(relativePath);
                
                // Load the imported asset
                GameObject objModel = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(relativePath);
                return objModel;
            }
            #else
            Debug.LogWarning("File loading is only supported in the Unity Editor. Please import your OBJ files into the project.");
            #endif
            
            return null;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error loading OBJ file: {e.Message}");
            return null;
        }
    }
    
    /// <summary>
    /// Converts absolute path to relative path within Unity project
    /// </summary>
    /// <param name="absolutePath">Absolute file path</param>
    /// <returns>Relative path from Assets folder</returns>
    private string GetRelativePath(string absolutePath)
    {
        string projectPath = Application.dataPath;
        if (absolutePath.StartsWith(projectPath))
        {
            return "Assets" + absolutePath.Substring(projectPath.Length);
        }
        
        // If file is outside project, copy it to Assets folder
        string fileName = Path.GetFileName(absolutePath);
        string targetPath = Path.Combine(projectPath, "ImportedModels", fileName);
        
        // Create directory if it doesn't exist
        Directory.CreateDirectory(Path.GetDirectoryName(targetPath));
        
        // Copy file to Assets folder
        File.Copy(absolutePath, targetPath, true);
        
        return "Assets/ImportedModels/" + fileName;
    }
    
    /// <summary>
    /// Spawns the 3D model at the origin (0,0,0)
    /// </summary>
    public void SpawnAtOrigin()
    {
        spawnPosition = Vector3.zero;
        Spawn3DModel();
    }
    
    /// <summary>
    /// Spawns the 3D model at the current camera position
    /// </summary>
    public void SpawnAtCamera()
    {
        if (Camera.main != null)
        {
            spawnPosition = Camera.main.transform.position + Camera.main.transform.forward * 2f;
            Spawn3DModel();
        }
        else
        {
            Debug.LogError("No main camera found!");
        }
    }
    
    /// <summary>
    /// Clears all spawned models from the scene
    /// </summary>
    public void ClearAllSpawnedModels()
    {
        GameObject[] spawnedModels = GameObject.FindGameObjectsWithTag("SpawnedModel");
        foreach (GameObject model in spawnedModels)
        {
            DestroyImmediate(model);
        }
        Debug.Log("Cleared all spawned models");
    }
    
    private void OnValidate()
    {
        // Ensure the button reference is valid
        if (selectFileButton == null)
        {
            selectFileButton = GetComponentInChildren<Button>();
        }
    }
    
    #region Windows File Dialog
    
    [DllImport("shell32.dll", CharSet = CharSet.Auto)]
    private static extern IntPtr ShellExecute(IntPtr hWnd, string lpOperation, string lpFile, string lpParameters, string lpDirectory, int nShowCmd);
    
    /// <summary>
    /// Opens a Windows file dialog to select a file
    /// </summary>
    /// <param name="title">Dialog title</param>
    /// <param name="extension">File extension filter (without dot)</param>
    /// <returns>Selected file path or empty string if cancelled</returns>
    private string OpenFileDialog(string title, string extension)
    {
        #if UNITY_EDITOR_WIN
        try
        {
            // Use Unity's built-in file dialog
            string path = UnityEditor.EditorUtility.OpenFilePanel(title, "", extension);
            return path;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error opening file dialog: {e.Message}");
            return "";
        }
        #else
        Debug.LogWarning("File dialog is only supported in Windows Unity Editor");
        return "";
        #endif
    }
    
    #endregion
}
