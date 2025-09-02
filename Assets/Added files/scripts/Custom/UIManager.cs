using UnityEngine;
using System.Collections.Generic;

public class UIManager : MonoBehaviour
{
    [Header("Panel Management")]
    [SerializeField] private GameObject[] allPanels; // All UI panels in the scene
    [SerializeField] private GameObject customUIPanel; // The custom UI panel
    [SerializeField] private string[] panelTags = { "UI_Panel", "Menu_Panel", "Settings_Panel" }; // Tags to identify panels
    
    private List<GameObject> managedPanels = new List<GameObject>();
    private bool isCustomUIOpen = false;
    
    void Awake()
    {
        InitializePanels();
    }
    
    private void InitializePanels()
    {
        managedPanels.Clear();
        
        // Add explicitly assigned panels
        if (allPanels != null)
        {
            foreach (GameObject panel in allPanels)
            {
                if (panel != null && !managedPanels.Contains(panel))
                {
                    managedPanels.Add(panel);
                }
            }
        }
        
        // Find panels by tags
        foreach (string tag in panelTags)
        {
            GameObject[] taggedPanels = GameObject.FindGameObjectsWithTag(tag);
            foreach (GameObject panel in taggedPanels)
            {
                if (panel != null && !managedPanels.Contains(panel))
                {
                    managedPanels.Add(panel);
                }
            }
        }
        
        // Find panels by common names
        string[] commonPanelNames = { "MainPanel", "ProgramPanel", "JogPanel", "SettingsPanel", "SavePanel", "LoadPanel" };
        foreach (string panelName in commonPanelNames)
        {
            GameObject panel = GameObject.Find(panelName);
            if (panel != null && !managedPanels.Contains(panel))
            {
                managedPanels.Add(panel);
            }
        }
    }
    
    public void OnCustomUIOpened()
    {
        isCustomUIOpen = true;
        HideAllOtherPanels();
    }
    
    public void OnCustomUIClosed()
    {
        isCustomUIOpen = false;
        // Optionally restore previously visible panels here
    }
    
    private void HideAllOtherPanels()
    {
        foreach (GameObject panel in managedPanels)
        {
            if (panel != null && panel != customUIPanel)
            {
                panel.SetActive(false);
            }
        }
    }
    
    public void ShowPanel(GameObject panel)
    {
        if (panel != null)
        {
            panel.SetActive(true);
            
            // If showing a panel other than custom UI, close custom UI
            if (panel != customUIPanel && isCustomUIOpen)
            {
                CloseCustomUI();
            }
        }
    }
    
    public void HidePanel(GameObject panel)
    {
        if (panel != null)
        {
            panel.SetActive(false);
        }
    }
    
    public void TogglePanel(GameObject panel)
    {
        if (panel != null)
        {
            bool isActive = panel.activeSelf;
            panel.SetActive(!isActive);
            
            // If toggling custom UI
            if (panel == customUIPanel)
            {
                if (!isActive)
                {
                    OnCustomUIOpened();
                }
                else
                {
                    OnCustomUIClosed();
                }
            }
            // If toggling any other panel and custom UI is open
            else if (!isActive && isCustomUIOpen)
            {
                CloseCustomUI();
            }
        }
    }
    
    private void CloseCustomUI()
    {
        if (customUIPanel != null)
        {
            customUIPanel.SetActive(false);
            OnCustomUIClosed();
        }
    }
    
    public void AddPanel(GameObject panel)
    {
        if (panel != null && !managedPanels.Contains(panel))
        {
            managedPanels.Add(panel);
        }
    }
    
    public void RemovePanel(GameObject panel)
    {
        if (panel != null && managedPanels.Contains(panel))
        {
            managedPanels.Remove(panel);
        }
    }
    
    public bool IsCustomUIOpen()
    {
        return isCustomUIOpen;
    }
    
    public List<GameObject> GetManagedPanels()
    {
        return new List<GameObject>(managedPanels);
    }
    
    // Method to refresh the panel list (useful if panels are created dynamically)
    public void RefreshPanelList()
    {
        InitializePanels();
    }
}
