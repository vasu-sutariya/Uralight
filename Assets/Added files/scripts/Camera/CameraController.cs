using UnityEngine;
using UnityEngine.EventSystems;

public class CameraController : MonoBehaviour
{
    [Header("Orbit Settings")]
    [SerializeField] private float orbitSpeed = 10f;
    [SerializeField] private float minVerticalAngle = -90f;
    [SerializeField] private float maxVerticalAngle = 90f;

    [Header("Pan Settings")]
    [SerializeField] private float basePanSpeed = 0.5f;
    [SerializeField] private float panZoomFactor = 0.1f;

    [Header("Zoom Settings")]
    [SerializeField] private float zoomSpeed = 4f;
    [SerializeField] private float orthographicZoomSpeed = 0.5f;
    [SerializeField] private float currentZoomDistance = 10f;
    [SerializeField] private float orthographicSize = 5f;
    //[SerializeField] private float minOrthographicSize = 0.001f;
    //[SerializeField] private float maxOrthographicSize = 20f;

    private Vector3 lastMousePosition;
    private float rotationX = 0f;
    private float rotationY = 0f;
    private Vector3 targetPosition;
    private Camera cam;
    private bool isOrthographic = false;
    
    // Animation variables for smooth view transitions
    private bool isAnimating = false;
    private float animationStartTime;
    private float animationDuration = 0.5f;
    private float startRotationX;
    private float startRotationY;
    private float targetRotationX;
    private float targetRotationY;

    private void Start()
    {
        // Initialize rotation angles based on current camera rotation
        Vector3 currentRotation = transform.eulerAngles;
        rotationX = currentRotation.y;
        rotationY = currentRotation.x;
        
        // Initialize zoom distance based on camera's current position
        currentZoomDistance = Vector3.Distance(transform.position, Vector3.zero);
        targetPosition = Vector3.zero;
        cam = GetComponent<Camera>();
        
        // Ensure camera can handle very small orthographic sizes
        cam.nearClipPlane = 0.0001f;
    }

    private void Update()
    {
        // Handle view animation
        if (isAnimating)
        {
            float elapsedTime = Time.time - animationStartTime;
            float progress = elapsedTime / animationDuration;
            
            if (progress >= 1f)
            {
                // Animation complete
                isAnimating = false;
                rotationX = targetRotationX;
                rotationY = targetRotationY;
                transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
            }
            else
            {
                // Smooth interpolation using ease-in-out
                float smoothProgress = Mathf.SmoothStep(0f, 1f, progress);
                rotationX = Mathf.LerpAngle(startRotationX, targetRotationX, smoothProgress);
                rotationY = Mathf.LerpAngle(startRotationY, targetRotationY, smoothProgress);
                transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
            }
        }
        else
        {
            // Right mouse button for orbit
            if (Input.GetMouseButton(1))
            {
                Vector3 mouseDelta = Input.mousePosition - lastMousePosition;
                
                // Perspective mode: Original orbiting behavior
                rotationX += mouseDelta.x * orbitSpeed * Time.deltaTime;
                rotationY -= mouseDelta.y * orbitSpeed * Time.deltaTime;
                rotationY = Mathf.Clamp(rotationY, minVerticalAngle, maxVerticalAngle);

                transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
            }
        }

        // Middle mouse button for pan
        if (Input.GetMouseButton(2))
        {
            Vector3 mouseDelta = Input.mousePosition - lastMousePosition;
            
            // Calculate pan speed based on zoom distance
            float zoomAdjustedPanSpeed = basePanSpeed * (currentZoomDistance * panZoomFactor);
            Vector3 moveDirection = new Vector3(-mouseDelta.x, -mouseDelta.y, 0) * zoomAdjustedPanSpeed * Time.deltaTime;
            
            // Transform the movement direction based on camera's orientation
            transform.Translate(moveDirection, Space.Self);
            targetPosition += transform.TransformDirection(moveDirection);
        }

        // Mouse scroll wheel for zoom
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");
        if (scrollInput != 0 && !IsPointerOverUI())
        {
            if (isOrthographic)
            {
                // Orthographic zoom (only adjust size)
                orthographicSize -= scrollInput * orthographicZoomSpeed;
                                
                cam.orthographicSize = orthographicSize;
            }
            else
            {
                // Perspective zoom (adjust distance)
                currentZoomDistance -= scrollInput * zoomSpeed;
                Vector3 direction = transform.rotation * Vector3.forward;
                transform.position = targetPosition - direction * currentZoomDistance;
            }
        }
        else if (!isOrthographic)
        {
            // Update camera position based on zoom (only in perspective mode)
            Vector3 direction = transform.rotation * Vector3.forward;
            transform.position = targetPosition - direction * currentZoomDistance;
        }

        lastMousePosition = Input.mousePosition;
    }

    public void SwitchToOrthographic()
    {
        if (!isOrthographic)
        {
            isOrthographic = true;
            cam.orthographic = true;
            cam.orthographicSize = orthographicSize;
        }
    }

    public void SwitchToPerspective()
    {
        if (isOrthographic)
        {
            isOrthographic = false;
            cam.orthographic = false;
        }
    }

    // Add these new methods for view gizmo navigation
    public void SetViewToFront()
    {
        StartViewAnimation(0f, 0f);
    }

    public void SetViewToBack()
    {
        StartViewAnimation(180f, 0f);
    }

    public void SetViewToRight()
    {
        StartViewAnimation(90f, 0f);
    }

    public void SetViewToLeft()
    {
        StartViewAnimation(-90f, 0f);
    }

    public void SetViewToTop()
    {
        StartViewAnimation(0f, 90f);
    }

    public void SetViewToBottom()
    {
        StartViewAnimation(0f, -90f);
    }

    public void SetViewToIso()
    {
        StartViewAnimation(45f, 45f);
    }

    private void StartViewAnimation(float targetX, float targetY)
    {
        // Store current rotation as starting point
        startRotationX = rotationX;
        startRotationY = rotationY;
        
        // Set target rotation
        targetRotationX = targetX;
        targetRotationY = targetY;
        
        // Start animation
        isAnimating = true;
        animationStartTime = Time.time;
    }

    public void UpdateRotationAngles(float x, float y)
    {
        rotationX = y;
        rotationY = x;
    }



    public float GetCurrentZoomDistance()
    {
        return currentZoomDistance;
    }

    public Vector3 GetTargetPosition()
    {
        return targetPosition;
    }

    private bool IsPointerOverUI()
    {
        // Check if the mouse pointer is over a UI element
        if (EventSystem.current == null)
            return false;
            
        return EventSystem.current.IsPointerOverGameObject();
    }

    // NEW: Set the camera's orbit pivot to a target's bounds center
    public void SetPivotTo(Transform targetTransform)
    {
        //Debug.Log("SetPivotTo: " + targetTransform.name);
        if (targetTransform == null)
            return;

        Vector3 pivot = GetHierarchyBoundsCenter(targetTransform.gameObject);
        targetPosition = pivot;

        // Update zoom distance to the new pivot and reposition the camera along its current forward
        currentZoomDistance = Vector3.Distance(transform.position, targetPosition);
        Vector3 direction = transform.rotation * Vector3.forward;
        transform.position = targetPosition - direction * currentZoomDistance;
    }

    private Vector3 GetHierarchyBoundsCenter(GameObject target)
    {
        var renderers = target.GetComponentsInChildren<Renderer>(true);
        if (renderers.Length == 0)
        {
            return target.transform.position;
        }

        Bounds combined = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++)
        {
            if (renderers[i] != null)
            {
                combined.Encapsulate(renderers[i].bounds);
            }
        }
        return combined.center;
    }
} 