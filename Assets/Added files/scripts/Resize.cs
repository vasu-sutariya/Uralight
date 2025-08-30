using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class Resize : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    [Header("Resize Settings")]
    [SerializeField] private float shrinkScale = 0.9f;
    [SerializeField] private float animationDuration = 0.1f;
    
    [Header("Button Control")]
    [SerializeField] private bool isButtonEnabled = true;
    
    private Button button;
    private RectTransform rectTransform;
    private Vector3 originalScale;
    private bool isHovering = false;
    private bool isAnimating = false;
    
    
    void Start()
    {
        // Get required components
        button = GetComponent<Button>();
        rectTransform = GetComponent<RectTransform>();
        
        if (button == null)
        {
            Debug.LogError("Resize script requires a Button component!");
            return;
        }
        
        if (rectTransform == null)
        {
            Debug.LogError("Resize script requires a RectTransform component!");
            return;
        }
        
        // Store the original scale
        originalScale = rectTransform.localScale;
        
        // Set initial button state
        SetButtonEnabled(isButtonEnabled);
    }
    
    public void OnPointerEnter(PointerEventData eventData)
    {
        // Only shrink if button is interactable
        if (button.interactable)
        {
            isHovering = true;
            if (!isAnimating)
            {
                StartCoroutine(AnimateScale(originalScale * shrinkScale));
            }
        }
    }
    
    public void OnPointerExit(PointerEventData eventData)
    {
        isHovering = false;
        if (!isAnimating)
        {
            StartCoroutine(AnimateScale(originalScale));
        }
    }
    
    private System.Collections.IEnumerator AnimateScale(Vector3 targetScale)
    {
        isAnimating = true;
        Vector3 startScale = rectTransform.localScale;
        float elapsedTime = 0f;
        
        while (elapsedTime < animationDuration)
        {
            // Check if button became disabled during animation
            if (!button.interactable && isHovering)
            {
                // If disabled while hovering, return to original size
                targetScale = originalScale;
                isHovering = false;
            }
            
            elapsedTime += Time.deltaTime;
            float progress = elapsedTime / animationDuration;
            
            // Use smooth interpolation
            rectTransform.localScale = Vector3.Lerp(startScale, targetScale, progress);
            
            yield return null;
        }
        
        // Ensure we reach the exact target scale
        rectTransform.localScale = targetScale;
        isAnimating = false;
    }
    
    // Optional: Method to reset scale if needed
    public void ResetScale()
    {
        if (rectTransform != null)
        {
            rectTransform.localScale = originalScale;
            isHovering = false;
            isAnimating = false;
        }
    }
    
    // Method to enable/disable the button
    public void SetButtonEnabled(bool enabled)
    {
        if (button != null)
        {
            button.interactable = enabled;
            isButtonEnabled = enabled;
            
            // If disabling while hovering, reset scale
            if (!enabled && isHovering)
            {
                ResetScale();
            }
        }
    }
    
    // Method to toggle button state
    public void ToggleButton()
    {
        SetButtonEnabled(!isButtonEnabled);
    }
    
    // Property to check if button is enabled
    public bool IsButtonEnabled
    {
        get { return isButtonEnabled; }
    }
}
