using UnityEngine;
using UnityEngine.UI;

public class DropZone : MonoBehaviour
{
    [Header("Drop Zone Settings")]
    [SerializeField] private bool canAcceptDrops = true;
    [SerializeField] private bool swapOnDrop = false; // If true, swap positions with dropped item
    [SerializeField] private Color highlightColor = Color.yellow;
    [SerializeField] private Color normalColor = Color.white;
    
    private Image image;
    private Color originalColor;
    private GameObject currentDroppedItem;
    
    void Awake()
    {
        image = GetComponent<Image>();
        if (image != null)
        {
            originalColor = image.color;
        }
    }
    
    public bool CanAcceptDrop(GameObject draggedObject)
    {
        if (!canAcceptDrops) return false;
        
        // You can add custom logic here to determine if this drop zone can accept the specific object
        // For example, check if it's a button, or has specific components, etc.
        
        return true; // For now, accept all drops
    }
    
    public void OnDrop(GameObject droppedObject)
    {
        if (!CanAcceptDrop(droppedObject)) return;
        
        currentDroppedItem = droppedObject;
        
        // Get the RectTransform components
        RectTransform droppedRect = droppedObject.GetComponent<RectTransform>();
        RectTransform thisRect = GetComponent<RectTransform>();
        
        if (droppedRect == null || thisRect == null) return;
        
        if (swapOnDrop)
        {
            // Swap positions with the dropped item
            SwapPositions(droppedRect, thisRect);
        }
        else
        {
            // Move the dropped item to this position
            droppedRect.SetParent(thisRect.parent);
            droppedRect.anchoredPosition = thisRect.anchoredPosition;
            droppedRect.SetSiblingIndex(thisRect.GetSiblingIndex());
        }
        
        // Reset highlight color
        ResetHighlight();
    }
    
    private void SwapPositions(RectTransform droppedRect, RectTransform thisRect)
    {
        // Store original values
        Vector2 droppedOriginalPosition = droppedRect.anchoredPosition;
        Transform droppedOriginalParent = droppedRect.parent;
        int droppedOriginalSiblingIndex = droppedRect.GetSiblingIndex();
        
        Vector2 thisOriginalPosition = thisRect.anchoredPosition;
        Transform thisOriginalParent = thisRect.parent;
        int thisOriginalSiblingIndex = thisRect.GetSiblingIndex();
        
        // Swap positions
        droppedRect.SetParent(thisOriginalParent);
        droppedRect.anchoredPosition = thisOriginalPosition;
        droppedRect.SetSiblingIndex(thisOriginalSiblingIndex);
        
        thisRect.SetParent(droppedOriginalParent);
        thisRect.anchoredPosition = droppedOriginalPosition;
        thisRect.SetSiblingIndex(droppedOriginalSiblingIndex);
    }
    
    public void Highlight()
    {
        if (image != null)
        {
            image.color = highlightColor;
        }
    }
    
    public void ResetHighlight()
    {
        if (image != null)
        {
            image.color = originalColor;
        }
    }
    
    // Public methods to configure the drop zone
    public void SetCanAcceptDrops(bool canAccept)
    {
        canAcceptDrops = canAccept;
    }
    
    public void SetSwapOnDrop(bool swap)
    {
        swapOnDrop = swap;
    }
    
    public void SetHighlightColor(Color color)
    {
        highlightColor = color;
    }
    
    public void SetNormalColor(Color color)
    {
        normalColor = color;
        originalColor = color;
        if (image != null)
        {
            image.color = color;
        }
    }
}
