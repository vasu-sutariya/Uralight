using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class Jog : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private TMP_InputField[] angleInputs = new TMP_InputField[6];
    [SerializeField] private TMP_InputField[] poseInputs = new TMP_InputField[6];
    [SerializeField] private Button moveButton; // Add this for the move button
    
    [Header("Robot Components")]
    [SerializeField] private UnityEncoder encoder;
    [SerializeField] private UnityJointController jointController;
    [SerializeField] private URInverseKinematics inverseKinematics;
    
    private float[] currentAngles = new float[6];
    private float[] currentPose = new float[6];
    private bool poseInputsBeingEdited = false; // Flag to track if pose inputs are being edited
    
    void Start()
    {
        // Add listeners to input fields
        for (int i = 0; i < 6; i++)
        {
            int index = i; // Capture index for lambda
            if (angleInputs != null && i < angleInputs.Length && angleInputs[i] != null)
            {
                angleInputs[i].onEndEdit.AddListener((value) => OnInputEndEdit(index, value));
            }
            if (poseInputs != null && i < poseInputs.Length && poseInputs[i] != null)
            {
                poseInputs[i].onSelect.AddListener((_) => OnPoseInputSelected());
                poseInputs[i].onDeselect.AddListener((_) => OnPoseInputDeselected());
                poseInputs[i].onEndEdit.AddListener((_) => OnPoseEndEdit());
            }
        }
        
        // Add listener to move button and set it inactive initially
        if (moveButton != null)
        {
            moveButton.onClick.AddListener(OnMoveButtonPressed);
            moveButton.gameObject.SetActive(false); // Start with button disabled
        }
    }
    
    void Update()
    {
        // Get current angles from encoder (degrees)
        currentAngles = encoder.GetUnityAngles();
        
        // Get current end-effector pose and cache it for UI display
        var pose = inverseKinematics.GetEndEffectorPose(currentAngles);
        currentPose[0] = pose.position.x;
        currentPose[1] = pose.position.y;
        currentPose[2] = pose.position.z;
        currentPose[3] = pose.rotation.x; // roll (deg)
        currentPose[4] = pose.rotation.y; // pitch (deg)
        currentPose[5] = pose.rotation.z; // yaw (deg)
        
        // Update display only when no input is focused and pose inputs are not being edited
        if (!AnyInputFocused() && !poseInputsBeingEdited)
        {
            UpdateDisplay();
        }
    }
    
    void UpdateDisplay()
    {
        for (int i = 0; i < 6; i++)
        {
            if (angleInputs != null && i < angleInputs.Length && angleInputs[i] != null)
            {
                angleInputs[i].text = currentAngles[i].ToString("F2");
            }
            if (poseInputs != null && i < poseInputs.Length && poseInputs[i] != null)
            {
                poseInputs[i].text = currentPose[i].ToString("F2");
            }
        }
    }
    
    bool AnyInputFocused()
    {
        for (int i = 0; i < 6; i++)
        {
            if (angleInputs != null && i < angleInputs.Length && angleInputs[i] != null && angleInputs[i].isFocused)
            {
                return true;
            }
            if (poseInputs != null && i < poseInputs.Length && poseInputs[i] != null && poseInputs[i].isFocused)
            {
                return true;
            }
        }
        return false;
    }
    
    void OnPoseInputSelected()
    {
        // Set flag when any pose input is selected/focused
        poseInputsBeingEdited = true;
        
        // Activate the move button when pose inputs are being edited
        if (moveButton != null)
        {
            moveButton.gameObject.SetActive(true);
        }
    }
    
    void OnPoseInputDeselected()
    {
        // Keep the flag true when deselected, as user might still be editing
        // The flag will be reset when move button is pressed
    }
    
    void OnInputEndEdit(int index, string value)
    {
        //Debug.Log("OnInputEndEdit: " + index + " " + value);
        
        if (float.TryParse(value, out float angle))
        {
            // Send degrees to joint controller
            float[] newAngles = new float[6];
            for (int i = 0; i < 6; i++)
            {
                newAngles[i] = i == index ? angle : currentAngles[i];
            }
            //Debug.Log("newAngles (deg): " + newAngles[0] + " " + newAngles[1] + " " + newAngles[2] + " " + newAngles[3] + " " + newAngles[4] + " " + newAngles[5]);
            jointController.ChangeUnityTargetAngles(newAngles);
        }
    }
    
    void OnPoseEndEdit()
    {
        // Don't execute movement here anymore - wait for move button
        //Debug.Log("Pose input editing ended - waiting for move button");
    }
    
    void OnMoveButtonPressed()
    {
        //Debug.Log("Move button pressed - executing pose movement");
        
        // Read 6 pose values: x,y,z, roll, pitch, yaw
        float[] poseVals = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (poseInputs == null || i >= poseInputs.Length || poseInputs[i] == null || !float.TryParse(poseInputs[i].text, out poseVals[i]))
            {
                //Debug.LogWarning("Invalid pose input at index " + i);
                return;
            }
        }

        Vector3 position = new Vector3(poseVals[0], poseVals[1], poseVals[2]);
        Vector3 rotation = new Vector3(poseVals[3], poseVals[4], poseVals[5]); // degrees
       
        //Debug.Log("position: " + position + " rotation: " + rotation);
        // Build desired transform and compute IK
        var solutions = inverseKinematics.CalculateIK(inverseKinematics.CreateDesiredTransform(position, rotation));
        if (solutions != null && solutions.Count > 0)
        {
            // Choose first solution (degrees)
            var anglesDeg = solutions[5];
            //Debug.Log("anglesDeg: " + anglesDeg[0] + " " + anglesDeg[1] + " " + anglesDeg[2] + " " + anglesDeg[3] + " " + anglesDeg[4] + " " + anglesDeg[5]);
            jointController.ChangeUnityTargetAngles(anglesDeg);
            
            // Reset the flag to resume updating pose inputs with actual values
            poseInputsBeingEdited = false;
            
            // Deactivate the move button after movement is executed
            if (moveButton != null)
            {
                moveButton.gameObject.SetActive(false);
            }
        }
        else
        {
            Debug.LogWarning("No IK solution found for given pose");
            
            // Reset the flag to resume updating pose inputs with actual values
            poseInputsBeingEdited = false;
            
            // Deactivate the move button since movement failed
            if (moveButton != null)
            {
                moveButton.gameObject.SetActive(false);
            }
        }
    }
}
