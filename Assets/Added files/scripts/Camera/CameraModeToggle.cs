using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class CameraModeToggle : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private Button toggleButton;
    [SerializeField] private TextMeshProUGUI orthographicText;
    [SerializeField] private TextMeshProUGUI perspectiveText;
    
    [Header("Camera Reference")]
    [SerializeField] private CameraController cameraController;
    
    private bool isOrthographic = false;
    
    private void Start()
    {
        // If camera controller is not assigned, try to find it
        if (cameraController == null)
        {
            cameraController = FindFirstObjectByType<CameraController>();
        }
        
        // Set up button click listener
        if (toggleButton != null)
        {
            toggleButton.onClick.AddListener(ToggleCameraMode);
        }
        
        // Initialize text states based on current camera mode
        UpdateTextDisplay();
    }
    
    public void ToggleCameraMode()
    {
        if (cameraController == null) return;
        
        isOrthographic = !isOrthographic;
        
        if (isOrthographic)
        {
            cameraController.SwitchToOrthographic();
        }
        else
        {
            cameraController.SwitchToPerspective();
        }
        
        UpdateTextDisplay();
    }
    
    private void UpdateTextDisplay()
    {
        if (orthographicText != null)
        {
            orthographicText.gameObject.SetActive(isOrthographic);
        }
        
        if (perspectiveText != null)
        {
            perspectiveText.gameObject.SetActive(!isOrthographic);
        }
    }
} 