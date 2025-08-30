using UnityEngine;

public class Gizmo : MonoBehaviour
{
    [SerializeField] private Camera mainCamera;
    [SerializeField] private Camera secondCamera;
    [SerializeField] private Transform targetFrame; // The frame/object to orbit around
    [SerializeField] private float orbitDistance = 10f;

    private void Start()
    {
        if (mainCamera == null)
            mainCamera = Camera.main;
    }

    private void Update()
    {
        if (mainCamera != null && secondCamera != null && targetFrame != null)
        {
            // Get the main camera's rotation
            Quaternion mainCameraRotation = mainCamera.transform.rotation;
            
            // Calculate the second camera's position based on the target frame and distance
            Vector3 direction = mainCameraRotation * Vector3.back; // Back because camera looks forward
            Vector3 newPosition = targetFrame.position + direction * orbitDistance;
            
            // Set the second camera's position and rotation
            secondCamera.transform.position = newPosition;
            secondCamera.transform.rotation = mainCameraRotation;
        }
    }
}
