using UnityEngine;
using System;
using System.Collections.Generic;
using System.Collections;

public enum MovementType
{
    MoveJ,  // Joint movement
    MoveL   // Linear movement
}

public class UnityTrajControl : MonoBehaviour
{
    [Header("Trajectory Settings")]
    public float maxAngularVelocity = 30f; // degrees per second
    public float maxAngularAcceleration = 60f; // degrees per second squared
    public float maxLinearVelocity = 0.5f; // meters per second
    public float maxLinearAcceleration = 1.0f; // meters per second squared
    
    [Header("Blend Settings")]
    public float defaultBlendRadius = 0.01f; // meters
    
    private float[] startAngles;
    private Matrix4x4 startMatrix;
    private float[] endAngles;
    private Matrix4x4 endMatrix;
    private Vector3 u;
    private float theta;
    private float[] shortestAngles = new float[6];
    
    private float totalTime;
    private float currentTime = 0f;
    private float[] currentAngles = new float[6];
    public bool isTrajectoryActive = false;
    
    // Timing variables for trajectory execution
    private float trajectoryStartTime;
    private float trajectoryExecutionTime;
    
    public UnityJointController robotController;
    public UnityEncoder encoder;
    public URInverseKinematics inverseKinematics; // For MoveL calculations
    public TrajectoryCalculator trajectoryCalculator;

    // Trajectory planning state
    private TrajectoryType currentTrajectoryType = TrajectoryType.Cubic;
    private bool currentTimeGiven = false;
    private float currentVelocity = 30f;
    private float currentAcceleration = 60f;

    private class MovementCommand
    {
        public float[] targetAngles;
        public float velocity;
        public float acceleration;
        public float blendRadius;
        public float time;
        public MovementType movementType;

        public MovementCommand(float[] angles, float vel, float acc, float blend, float t, MovementType type)
        {
            targetAngles = angles;
            velocity = vel;
            acceleration = acc;
            blendRadius = blend;
            time = t;
            movementType = type;
        }
    }

    void Start()
    {
        // Initialize currentAngles with the robot's actual position
        currentAngles = encoder.GetUnityAngles(); 
    }

    void Update()
    {
        if (isTrajectoryActive)
        {
            currentTime += Time.deltaTime;
            
            if (currentTime < totalTime)
            {
                if (currentTrajectoryType == TrajectoryType.LinearCubic || currentTrajectoryType == TrajectoryType.LinearTrapezoidal)
                {
                    float[] currentAnglesandangle = trajectoryCalculator.CalculateLinearTrajectory(startMatrix, endMatrix, u, theta, currentTime, totalTime, currentTrajectoryType, currentTimeGiven, currentVelocity, currentAcceleration);
                    
                    if (currentAnglesandangle[0] == 1000f)
                    {
                        isTrajectoryActive = false;
                        return;
                    }
                    else
                    {
                    currentAngles[0] = currentAnglesandangle[0];
                    currentAngles[1] = currentAnglesandangle[1];
                    currentAngles[2] = currentAnglesandangle[2];
                    currentAngles[3] = currentAnglesandangle[3];
                    currentAngles[4] = currentAnglesandangle[4];
                    currentAngles[5] = currentAnglesandangle[5];
                    }
                }
                
                else
                {
                    for (int i = 0; i < 6; i++)
                    {
                    currentAngles[i] = trajectoryCalculator.CalculateTrajectory(
                        startAngles[i], 
                        shortestAngles[i], 
                        currentTime, 
                        totalTime,
                        currentTrajectoryType,
                        currentTimeGiven,
                        currentVelocity,
                        currentAcceleration
                    );
                }
                }
                
                robotController.ChangeUnityTargetAngles(currentAngles);
            }
            else
            {
                //robotController.ChangeUnityTargetAngles(endAngles);
                isTrajectoryActive = false;
                
                // Calculate and display trajectory execution time
                trajectoryExecutionTime = Time.time - trajectoryStartTime;
                //Debug.Log($"Trajectory completed! Execution time: {trajectoryExecutionTime:F3} seconds");
                
                //Debug.Log($"Movement Complete: Final Angles=[{string.Join(", ", endAngles)}]");
            }
        }
    }
    
    public void Goto(float[] target, MovementType movementType = MovementType.MoveJ, float velocity = -1f, float acceleration = -1f, float blendRadius = -1f, float time = -1f)
    {
        // Use default values if not specified
        float vel = velocity < 0 ? (movementType == MovementType.MoveJ ? maxAngularVelocity : maxLinearVelocity) : velocity;
        float acc = acceleration < 0 ? (movementType == MovementType.MoveJ ? maxAngularAcceleration : maxLinearAcceleration) : acceleration;
        float blend = blendRadius < 0 ? defaultBlendRadius : blendRadius;
        
        // Get current angles from encoder
        currentAngles = encoder.GetUnityAngles();
        
        startAngles = new float[6];
        startMatrix = new UnityEngine.Matrix4x4();
        System.Array.Copy(currentAngles, startAngles, 6);
        
        
        float maxDistance = 0f;
        
        if (movementType == MovementType.MoveJ)
        {
            endAngles = target;
            
            // For MoveJ: Calculate angular distance in degrees
            for (int i = 0; i < 6; i++)
            {
                shortestAngles[i] = ((endAngles[i] - startAngles[i] + 540) % 360) - 180;
                float absDistance = Mathf.Abs(shortestAngles[i]);
                maxDistance = Mathf.Max(maxDistance, absDistance);
            }
        }
        else if (movementType == MovementType.MoveL)
        {
            // For MoveL: Calculate linear distance in meters
            var currentPose = inverseKinematics.GetEndEffectorPose(currentAngles);
            startMatrix = inverseKinematics.PoseToTransform(currentPose.position.x, currentPose.position.y, currentPose.position.z, currentPose.rotation.x, currentPose.rotation.y, currentPose.rotation.z);
            
            AXisangle.MatrixToRPY(startMatrix, out Vector3 rpyGot);
            Debug.Log($"rpy start = {rpyGot.x}, {rpyGot.y}, {rpyGot.z}");
            
            endMatrix = inverseKinematics.PoseToTransform(target[0], target[1], target[2], target[3], target[4], target[5]);

            AXisangle.MatrixToRPY(endMatrix, out Vector3 rpyGotEnd);
            Debug.Log($"rpy = {rpyGotEnd.x}, {rpyGotEnd.y}, {rpyGotEnd.z}");

            UnityEngine.Matrix4x4 Rif  =    UnityEngine.Matrix4x4.Transpose(startMatrix) * endMatrix;   
            
            AXisangle.MatrixToAxisAngle(Rif, out Vector3 uGot, out float thetaGot);
            Debug.Log($"u = {uGot.x}, {uGot.y}, {uGot.z}, theta = {thetaGot}");
            u = uGot;
            theta = thetaGot;
            Debug.Log($"u = {u.x}, {u.y}, {u.z}, theta = {theta}");


        }
        
        // Handle only 2 trajectory planning cases for MoveJ
        bool timeGiven = time > 0;
        Debug.Log($"timeGiven: {timeGiven}");
        // Set trajectory state variables
        currentTimeGiven = timeGiven;
        currentVelocity = vel;
        currentAcceleration = acc;
        
        if (movementType == MovementType.MoveJ)
        {
            if (timeGiven)
            {
                // Case 1: MOVEJ - time given, blend radius 0 (ignore max vel, max acc)
                totalTime = time;
                currentTrajectoryType = TrajectoryType.Cubic; // Use cubic trajectory with given time
                Debug.Log($"Case 1: MOVEJ with cubic trajectory, explicit time={time:F2}s, no blending");
            }
            else
            {
                // Case 3: MOVEJ - time 0, blend radius 0
                totalTime = trajectoryCalculator.CalculateTrapezoidalTime(maxDistance, vel, acc);
                currentTrajectoryType = TrajectoryType.Trapezoidal; // Use trapezoidal trajectory
                Debug.Log($"Case 3: MOVEJ with trapezoidal trajectory, calculated time={totalTime:F2}s, no blending");
            }
        
        }
        else
        {
            if (timeGiven)
            {
                totalTime = time;
                currentTrajectoryType = TrajectoryType.LinearCubic; // Use cubic trajectory with given time
                Debug.Log($"Case 5: MOVEL with cubic trajectory, explicit time={time:F2}s, no blending");
            }
            else
            {
                
            }
        }
        
        currentTime = 0f;
        isTrajectoryActive = true;
        
        // Reset solution selection for new trajectory
        trajectoryCalculator.ResetSolutionSelection();
        
        // Record trajectory start time
        trajectoryStartTime = Time.time;
        //Debug.Log($"Trajectory started at: {trajectoryStartTime:F3} seconds");
        
        //Debug.Log($"Starting {movementType} trajectory: Distance={maxDistance:F2}°, Velocity={vel:F2}°/s, Acceleration={acc:F2}°/s², Time={totalTime:F2}s");
    }

    // Convenience methods for MoveJ and MoveL
    public void MoveJ(float[] targetAngles, float velocity = -1f, float acceleration = -1f, float blendRadius = -1f, float time = -1f)
    {
        //Debug.Log($"MoveJ: {targetAngles}, {velocity}, {acceleration}, {blendRadius}, {time}");     
        Goto(targetAngles, MovementType.MoveJ, velocity, acceleration, blendRadius, time);
    }
    
    public void MoveL(float[] targetPose, float velocity = -1f, float acceleration = -1f, float blendRadius = -1f, float time = -1f)
    {
        //Debug.Log($"MoveL: {targetPose}, {velocity}, {acceleration}, {blendRadius}, {time}");
        Goto(targetPose, MovementType.MoveL, velocity, acceleration, blendRadius, time);
    }
}
