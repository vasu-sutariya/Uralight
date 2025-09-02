using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class DragHandler : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    [Header("Drag Settings")]
    [SerializeField] private bool isDraggable = true;
    [SerializeField] private Canvas parentCanvas;
    [SerializeField] private GraphicRaycaster graphicRaycaster;
    
    private RectTransform rectTransform;
    private CanvasGroup canvasGroup;
    private Vector2 originalPosition;
    private Transform originalParent;
    private int originalSiblingIndex;
    private bool isDragging = false;
    
    void Awake()
    {
        rectTransform = GetComponent<RectTransform>();
        canvasGroup = GetComponent<CanvasGroup>();
        
        // If no canvas group, add one
        if (canvasGroup == null)
        {
            canvasGroup = gameObject.AddComponent<CanvasGroup>();
        }
        
        // Find parent canvas if not assigned
        if (parentCanvas == null)
        {
            parentCanvas = GetComponentInParent<Canvas>();
        }
        
        // Find graphic raycaster if not assigned
        if (graphicRaycaster == null && parentCanvas != null)
        {
            graphicRaycaster = parentCanvas.GetComponent<GraphicRaycaster>();
        }
    }
    
    public void OnBeginDrag(PointerEventData eventData)
    {
        if (!isDraggable) return;
        
        isDragging = true;
        
        // Store original position and parent
        originalPosition = rectTransform.anchoredPosition;
        originalParent = rectTransform.parent;
        originalSiblingIndex = rectTransform.GetSiblingIndex();
        
        // Make the button semi-transparent while dragging
        if (canvasGroup != null)
        {
            canvasGroup.alpha = 0.6f;
            canvasGroup.blocksRaycasts = false;
        }
        
        // Move to top of hierarchy for proper rendering
        rectTransform.SetAsLastSibling();
    }
    
    public void OnDrag(PointerEventData eventData)
    {
        if (!isDraggable || !isDragging) return;
        
        // Convert screen position to local position within the canvas
        Vector2 localPoint;
        if (RectTransformUtility.ScreenPointToLocalPointInRectangle(
            parentCanvas.transform as RectTransform,
            eventData.position,
            eventData.pressEventCamera,
            out localPoint))
        {
            rectTransform.anchoredPosition = localPoint;
        }
    }
    
    public void OnEndDrag(PointerEventData eventData)
    {
        if (!isDraggable || !isDragging) return;
        
        isDragging = false;
        
        // Restore visual properties
        if (canvasGroup != null)
        {
            canvasGroup.alpha = 1f;
            canvasGroup.blocksRaycasts = true;
        }
        
        // Check if we're dropping on a valid drop zone
        GameObject dropTarget = GetDropTarget(eventData);
        
        if (dropTarget != null)
        {
            // Handle successful drop
            HandleSuccessfulDrop(dropTarget);
        }
        else
        {
            // Return to original position if no valid drop target
            ReturnToOriginalPosition();
        }
    }
    
    private GameObject GetDropTarget(PointerEventData eventData)
    {
        // Create a list to store raycast results
        var results = new System.Collections.Generic.List<RaycastResult>();
        
        // Perform the raycast
        if (graphicRaycaster != null)
        {
            graphicRaycaster.Raycast(eventData, results);
        }
        
        // Check each result for a valid drop target
        foreach (var result in results)
        {
            GameObject target = result.gameObject;
            
            // Skip self
            if (target == gameObject) continue;
            
            // Check if it's a valid drop zone (you can customize this logic)
            DropZone dropZone = target.GetComponent<DropZone>();
            if (dropZone != null && dropZone.CanAcceptDrop(gameObject))
            {
                return target;
            }
            
            // Also allow dropping on other buttons or UI elements
            if (target.GetComponent<Button>() != null || target.GetComponent<Image>() != null)
            {
                return target;
            }
        }
        
        return null;
    }
    
    private void HandleSuccessfulDrop(GameObject dropTarget)
    {
        // Get the drop zone component if it exists
        DropZone dropZone = dropTarget.GetComponent<DropZone>();
        
        if (dropZone != null)
        {
            // Let the drop zone handle the drop
            dropZone.OnDrop(gameObject);
        }
        else
        {
            // Default behavior: move to the drop target's position
            RectTransform targetRect = dropTarget.GetComponent<RectTransform>();
            if (targetRect != null)
            {
                rectTransform.SetParent(targetRect.parent);
                rectTransform.anchoredPosition = targetRect.anchoredPosition;
                rectTransform.SetSiblingIndex(targetRect.GetSiblingIndex());
            }
        }
    }
    
    private void ReturnToOriginalPosition()
    {
        // Return to original parent and position
        rectTransform.SetParent(originalParent);
        rectTransform.SetSiblingIndex(originalSiblingIndex);
        rectTransform.anchoredPosition = originalPosition;
    }
    
    // Public method to enable/disable dragging
    public void SetDraggable(bool draggable)
    {
        isDraggable = draggable;
        
        // If we're currently dragging and dragging is disabled, end the drag
        if (!draggable && isDragging)
        {
            ReturnToOriginalPosition();
            isDragging = false;
            
            if (canvasGroup != null)
            {
                canvasGroup.alpha = 1f;
                canvasGroup.blocksRaycasts = true;
            }
        }
    }
    
    // Public method to check if currently dragging
    public bool IsDragging()
    {
        return isDragging;
    }
}
