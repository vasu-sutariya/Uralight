using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class Axis_set : MonoBehaviour, IPointerClickHandler, IPointerDownHandler, IPointerUpHandler
{
    [Header("References")]
    [SerializeField] private RawImage targetRawImage;
    [SerializeField] private Canvas parentCanvas;
    
    [Header("Ray Casting")]
    [SerializeField] private Camera rayCamera; // The second camera to cast rays from
    [SerializeField] private float rayDistance = 100f; // How far the ray should go
    [SerializeField] private LayerMask raycastLayerMask = -1; // Which layers to hit
    
    [Header("Camera Controller")]
    [SerializeField] private CameraController cameraController; // Reference to the camera controller
    
    [Header("Ray Visualization")]
    [SerializeField] private bool showRay = true;
    [SerializeField] private Color rayColor = Color.red;
    [SerializeField] private float rayWidth = 0.1f;
    [SerializeField] private float rayDuration = 2f; // How long to show the ray
    
    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = true;
    [SerializeField] private bool useAlternativeClickDetection = false;
    
    private RectTransform rawImageRectTransform;
    private Camera uiCamera;
    private bool isInitialized = false;
    private LineRenderer rayLineRenderer;
    private float rayTimer = 0f;
    
    void Start()
    {
        InitializeReferences();
        SetupRayVisualization();
    }
    
    void InitializeReferences()
    {
        // Get references if not assigned
        if (targetRawImage == null)
            targetRawImage = GetComponent<RawImage>();
            
        if (parentCanvas == null)
            parentCanvas = GetComponentInParent<Canvas>();
            
        if (targetRawImage != null)
            rawImageRectTransform = targetRawImage.GetComponent<RectTransform>();
            
        // Get UI camera (for Screen Space - Camera mode)
        if (parentCanvas != null)
        {
            if (parentCanvas.renderMode == RenderMode.ScreenSpaceCamera)
            {
                uiCamera = parentCanvas.worldCamera;
            }
            else if (parentCanvas.renderMode == RenderMode.ScreenSpaceOverlay)
            {
                // For Screen Space - Overlay, we don't need a camera
                uiCamera = null;
            }
            else if (parentCanvas.renderMode == RenderMode.WorldSpace)
            {
                // For World Space, we need a camera
                uiCamera = Camera.main;
            }
        }
            
        isInitialized = true;
        
        if (showDebugInfo)
        {
            Debug.Log($"Axis_set initialized:");
            Debug.Log($"- Target RawImage: {(targetRawImage != null ? "Found" : "NOT FOUND")}");
            Debug.Log($"- Parent Canvas: {(parentCanvas != null ? "Found" : "NOT FOUND")}");
            Debug.Log($"- RawImage RectTransform: {(rawImageRectTransform != null ? "Found" : "NOT FOUND")}");
            Debug.Log($"- Canvas Render Mode: {(parentCanvas != null ? parentCanvas.renderMode.ToString() : "No Canvas")}");
            Debug.Log($"- UI Camera: {(uiCamera != null ? "Found" : "NOT FOUND")} ({(parentCanvas != null ? parentCanvas.renderMode.ToString() : "No Canvas")})");
            Debug.Log($"- Ray Camera: {(rayCamera != null ? "Found" : "NOT FOUND")}");
            Debug.Log($"- Camera Controller: {(cameraController != null ? "Found" : "NOT FOUND")}");
        }
    }
    
    void SetupRayVisualization()
    {
        // Create LineRenderer for ray visualization
        if (showRay)
        {
            rayLineRenderer = gameObject.AddComponent<LineRenderer>();
            rayLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            rayLineRenderer.startColor = rayColor;
            rayLineRenderer.endColor = rayColor;
            rayLineRenderer.startWidth = rayWidth;
            rayLineRenderer.endWidth = rayWidth;
            rayLineRenderer.positionCount = 2;
            rayLineRenderer.enabled = false;
        }
    }
    
    // Method 1: Using IPointerClickHandler (requires script on RawImage)
    public void OnPointerClick(PointerEventData eventData)
    {
        if (showDebugInfo)
            Debug.Log("OnPointerClick triggered!");
            
        HandleClick(eventData.position);
    }
    
    // Method 2: Using IPointerDownHandler
    public void OnPointerDown(PointerEventData eventData)
    {
        if (showDebugInfo)
            Debug.Log("OnPointerDown triggered!");
            
        if (useAlternativeClickDetection)
            HandleClick(eventData.position);
    }
    
    // Method 3: Using IPointerUpHandler
    public void OnPointerUp(PointerEventData eventData)
    {
        if (showDebugInfo)
            Debug.Log("OnPointerUp triggered!");
    }
    
    private void HandleClick(Vector2 screenPosition)
    {
        if (!isInitialized)
        {
            Debug.LogError("Script not initialized properly!");
            return;
        }
        
        if (targetRawImage == null || rawImageRectTransform == null)
        {
            Debug.LogError("RawImage or RectTransform not found!");
            return;
        }
        
        Vector2 localPosition = GetLocalPositionOnRawImage(screenPosition);
        
        if (showDebugInfo)
        {
            Debug.Log($"Clicked on RawImage at local position: {localPosition}");
            Debug.Log($"Screen position: {screenPosition}");
        }
        
        // Cast ray from the second camera
        CastRayFromCamera(localPosition);
        
        // You can use localPosition here for your specific needs
        OnRawImageClicked(localPosition);
    }
    
    private void CastRayFromCamera(Vector2 localPosition)
    {
        if (rayCamera == null)
        {
            Debug.LogError("Ray Camera not assigned!");
            return;
        }
        
        // Convert local position to normalized position (0-1)
        Vector2 normalizedPosition = GetNormalizedPosition(Input.mousePosition);
        
        // Create a ray from the camera through the normalized position
        Ray ray = rayCamera.ViewportPointToRay(normalizedPosition);
        
        if (showDebugInfo)
        {
            Debug.Log($"Casting ray from camera at normalized position: {normalizedPosition}");
            Debug.Log($"Ray origin: {ray.origin}, Ray direction: {ray.direction}");
        }
        
        // Perform the raycast
        RaycastHit hit;
        bool hitSomething = Physics.Raycast(ray, out hit, rayDistance, raycastLayerMask);
        
        if (hitSomething)
        {
            if (showDebugInfo)
            {
                Debug.Log($"Ray hit: {hit.collider.name} at position {hit.point}");
                Debug.Log($"Hit distance: {hit.distance}");
            }
            
            // Check if the hit object is one of our axis objects
            HandleAxisObjectHit(hit.collider.gameObject);
            
            // Visualize the ray from camera to hit point
            VisualizeRay(ray.origin, hit.point);
        }
        else
        {
            if (showDebugInfo)
                Debug.Log("Ray didn't hit anything");
                
            // Visualize the ray from camera to max distance
            Vector3 endPoint = ray.origin + ray.direction * rayDistance;
            VisualizeRay(ray.origin, endPoint);
        }
    }
    
    private void VisualizeRay(Vector3 start, Vector3 end)
    {
        if (!showRay || rayLineRenderer == null)
            return;
            
        rayLineRenderer.enabled = true;
        rayLineRenderer.SetPosition(0, start);
        rayLineRenderer.SetPosition(1, end);
        
        // Start timer to hide ray after duration
        rayTimer = rayDuration;
    }
    
    private Vector2 GetLocalPositionOnRawImage(Vector2 screenPosition)
    {
        Vector2 localPosition;
        
        // Convert screen position to local position on the RawImage
        bool success = RectTransformUtility.ScreenPointToLocalPointInRectangle(
            rawImageRectTransform, 
            screenPosition, 
            uiCamera, 
            out localPosition
        );
        
        if (!success && showDebugInfo)
        {
            Debug.LogWarning("Failed to convert screen point to local point!");
        }
        
        return localPosition;
    }
    
    // Method to get normalized position (0-1 range)
    public Vector2 GetNormalizedPosition(Vector2 screenPosition)
    {
        Vector2 localPosition = GetLocalPositionOnRawImage(screenPosition);
        
        // Convert to normalized coordinates (0-1)
        Vector2 normalizedPosition = new Vector2(
            (localPosition.x + rawImageRectTransform.rect.width * 0.5f) / rawImageRectTransform.rect.width,
            (localPosition.y + rawImageRectTransform.rect.height * 0.5f) / rawImageRectTransform.rect.height
        );
        
        return normalizedPosition;
    }
    
    // Override this method to handle the click event
    protected virtual void OnRawImageClicked(Vector2 localPosition)
    {
        // Override this method in derived classes or use events
        //Debug.Log($"RawImage clicked at local position: {localPosition}");
    }
    
    // Alternative method using Update for continuous input (if needed)
    void Update()
    {
        // Handle ray visualization timer
        if (rayTimer > 0)
        {
            rayTimer -= Time.deltaTime;
            if (rayTimer <= 0 && rayLineRenderer != null)
            {
                rayLineRenderer.enabled = false;
            }
        }
        
        // Alternative click detection using Input system
        if (Input.GetMouseButtonDown(0))
        {
            if (showDebugInfo)
                Debug.Log("Mouse button down detected in Update");
                
            // Check if we're clicking on this RawImage
            if (IsMouseOverRawImage())
            {
                if (showDebugInfo)
                    Debug.Log("Mouse is over RawImage!");
                    
                HandleClick(Input.mousePosition);
            }
        }
    }
    
    private bool IsMouseOverRawImage()
    {
        if (targetRawImage == null || rawImageRectTransform == null)
            return false;
            
        // Check if mouse is over this UI element
        return RectTransformUtility.RectangleContainsScreenPoint(
            rawImageRectTransform, 
            Input.mousePosition, 
            uiCamera
        );
    }
    
    
    
    // Handle hits on axis objects and call appropriate camera controller functions
    private void HandleAxisObjectHit(GameObject hitObject)
    {
        
        string objectName = hitObject.name.ToLower();
        
        
        // Check for axis objects and call appropriate camera controller functions
        switch (objectName)
        {
            case "plusx":
                if (showDebugInfo)
                    Debug.Log("Hit plusx - calling SetViewToRight()");
                cameraController.SetViewToLeft();
                break;
                
            case "minusx":
                if (showDebugInfo)
                    Debug.Log("Hit minusx - calling SetViewToLeft()");
                cameraController.SetViewToRight();
                break;
                
            case "plusy":
                if (showDebugInfo)
                    Debug.Log("Hit plusy - calling SetViewToTop()");
                cameraController.SetViewToBack();
                break;
                
            case "minusy":
                if (showDebugInfo)
                    Debug.Log("Hit minusy - calling SetViewToBottom()");
                cameraController.SetViewToFront();
                break;
                
            case "plusz":
                if (showDebugInfo)
                    Debug.Log("Hit plusz - calling SetViewToFront()");
                cameraController.SetViewToTop();
                break;
                
            case "minusz":
                if (showDebugInfo)
                    Debug.Log("Hit minusz - calling SetViewToBack()");
                cameraController.SetViewToBottom();
                break;
                
            default:
                if (showDebugInfo)
                    Debug.Log($"Hit object '{objectName}' - no specific action defined");
                break;
        }
    }
    
    
}
