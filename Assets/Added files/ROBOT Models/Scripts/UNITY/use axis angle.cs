using UnityEngine;

public class useAxisAngle : MonoBehaviour
{
    [Header("Input RPY (degrees)")]
    public Vector3 rpyDegrees = new Vector3(30f, 45f, 60f);
    public bool Debugger = false;
    
    [Header("Results - Degrees")]
    [SerializeField] private Vector3 rotationVectorDegrees;
    [SerializeField] private Vector3 rpyBackFromRotVecDegrees;
    
    [Header("Results - Radians")]
    [SerializeField] private Vector3 rpyRadians;
    [SerializeField] private Vector3 rotationVectorRadians;
    [SerializeField] private Vector3 rpyBackFromRotVecRadians;
    
    [Header("Axis-Angle Representation")]
    [SerializeField] private Vector3 rotationAxis;
    [SerializeField] private float rotationAngleDegrees;

    void Start()
    {
        DemonstrateConversions();
    }

    void Update()
    {
        // Update conversions when RPY input changes in inspector
        if (Debugger)
        {
            DemonstrateConversions();
        }
    }
    
    void DemonstrateConversions()
    {
        // Convert RPY degrees to rotation vector degrees
        rotationVectorDegrees = AXisangle.RPYDegreesToRotationVector(rpyDegrees);
        
        // Convert rotation vector back to RPY degrees (round trip test)
        rpyBackFromRotVecDegrees = AXisangle.RotationVectorToRPYDegrees(rotationVectorDegrees);
        
        // Convert RPY degrees to radians
        rpyRadians = rpyDegrees * Mathf.Deg2Rad;
        
        // Convert RPY radians to rotation vector radians
        rotationVectorRadians = AXisangle.RPYRadiansToRotationVector(rpyRadians);
        
        // Convert rotation vector radians back to RPY radians (round trip test)
        rpyBackFromRotVecRadians = AXisangle.RotationVectorToRPYRadians(rotationVectorRadians);
        
        // Get axis-angle representation
        AXisangle.RPYDegreesToAxisAngle(rpyDegrees, out rotationAxis, out rotationAngleDegrees);
        
        // Debug output
        Debug.Log($"RPY Degrees: {rpyDegrees}");
        Debug.Log($"Rotation Vector Degrees: {rotationVectorDegrees}");
        Debug.Log($"RPY Radians: {rpyRadians}");
        Debug.Log($"Rotation Vector Radians: {rotationVectorRadians}");
        Debug.Log($"Axis: {rotationAxis}, Angle: {rotationAngleDegrees}°");
        Debug.Log($"Round-trip RPY Degrees: {rpyBackFromRotVecDegrees}");
        Debug.Log($"Round-trip RPY Radians: {rpyBackFromRotVecRadians * Mathf.Rad2Deg}°");
    }
}
