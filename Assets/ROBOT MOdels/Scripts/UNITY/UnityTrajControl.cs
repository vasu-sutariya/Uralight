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
    private float[] endAngles;
    private float[] shortestAngles = new float[6];
    
    private float totalTime;
    private float currentTime = 0f;
    private float[] currentAngles = new float[6];
    public bool isTrajectoryActive = false;
    public UnityJointController robotController;
    public UnityEncoder encoder;
    public URInverseKinematics inverseKinematics; // For MoveL calculations

    // Trajectory planning state
    private TrajectoryType currentTrajectoryType = TrajectoryType.Cubic;
    private bool currentTimeGiven = false;
    private bool currentBlendEnabled = false;
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
                for (int i = 0; i < 6; i++)
                {
                    currentAngles[i] = TrajectoryCalculator.CalculateTrajectory(
                        startAngles[i], 
                        shortestAngles[i], 
                        currentTime, 
                        totalTime,
                        currentTrajectoryType,
                        currentTimeGiven,
                        currentBlendEnabled,
                        currentVelocity,
                        currentAcceleration
                    );
                }
                robotController.ChangeUnityTargetAngles(currentAngles);
            }
            else
            {
                //robotController.ChangeUnityTargetAngles(endAngles);
                isTrajectoryActive = false;
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
            
        }
        
        // Handle 8 different trajectory planning cases
        bool timeGiven = time > 0;
        bool blendEnabled = blend > 0;
        
        // Set trajectory state variables
        currentTimeGiven = timeGiven;
        currentBlendEnabled = blendEnabled;
        currentVelocity = vel;
        currentAcceleration = acc;
        
        if (movementType == MovementType.MoveJ)
        {
            if (timeGiven)
            {
                if (!blendEnabled)
                {
                    // Case 1: MOVEJ - time given, blend radius 0 (ignore max vel, max acc)
                    totalTime = time;
                    currentTrajectoryType = TrajectoryType.Cubic; // Use cubic trajectory with given time
                    Debug.Log($"Case 1: MOVEJ with cubic trajectory, explicit time={time:F2}s, no blending");
                }
                else
                {
                    // Case 2: MOVEJ - time given, blend radius not 0 (ignore max vel, max acc)
                    totalTime = time;
                    currentTrajectoryType = TrajectoryType.Cubic; // Can be changed to appropriate type
                    Debug.Log($"Case 2: MOVEJ with explicit time={time:F2}s, blend radius={blend:F3}m");
                }
            }
            else
            {
                if (!blendEnabled)
                {
                    // Case 3: MOVEJ - time 0, blend radius 0
                    totalTime = TrajectoryCalculator.CalculateTrapezoidalTime(maxDistance, vel, acc);
                    currentTrajectoryType = TrajectoryType.Trapezoidal; // Use trapezoidal trajectory
                    Debug.Log($"Case 3: MOVEJ with trapezoidal trajectory, calculated time={totalTime:F2}s, no blending");
                }
                else
                {
                    // Case 4: MOVEJ - time 0, blend radius not 0
                    totalTime = TrajectoryCalculator.CalculateRequiredTime(maxDistance, vel, acc);
                    currentTrajectoryType = TrajectoryType.Cubic; // Can be changed to appropriate type
                    Debug.Log($"Case 4: MOVEJ with calculated time={totalTime:F2}s, blend radius={blend:F3}m");
                }
            }
        }
        else if (movementType == MovementType.MoveL)
        {
            if (timeGiven)
            {
                if (!blendEnabled)
                {
                    // Case 5: MOVEL - time given, blend radius 0 (ignore max vel, max acc)
                    totalTime = time;
                    currentTrajectoryType = TrajectoryType.Linear; // Linear for MoveL
                    Debug.Log($"Case 5: MOVEL with explicit time={time:F2}s, no blending");
                }
                else
                {
                    // Case 6: MOVEL - time given, blend radius not 0 (ignore max vel, max acc)
                    totalTime = time;
                    currentTrajectoryType = TrajectoryType.Linear; // Linear for MoveL
                    Debug.Log($"Case 6: MOVEL with explicit time={time:F2}s, blend radius={blend:F3}m");
                }
            }
            else
            {
                if (!blendEnabled)
                {
                    // Case 7: MOVEL - time 0, blend radius 0
                    totalTime = TrajectoryCalculator.CalculateRequiredTime(maxDistance, vel, acc);
                    currentTrajectoryType = TrajectoryType.Linear; // Linear for MoveL
                    Debug.Log($"Case 7: MOVEL with calculated time={totalTime:F2}s, no blending");
                }
                else
                {
                    // Case 8: MOVEL - time 0, blend radius not 0
                    totalTime = TrajectoryCalculator.CalculateRequiredTime(maxDistance, vel, acc);
                    currentTrajectoryType = TrajectoryType.Linear; // Linear for MoveL
                    Debug.Log($"Case 8: MOVEL with calculated time={totalTime:F2}s, blend radius={blend:F3}m");
                }
            }
        }
        
        currentTime = 0f;
        isTrajectoryActive = true;
        Debug.Log($"Starting {movementType} trajectory: Distance={maxDistance:F2}°, Velocity={vel:F2}°/s, Acceleration={acc:F2}°/s², Time={totalTime:F2}s, Blend={blend:F3}m");
    }

    // Convenience methods for MoveJ and MoveL
    public void MoveJ(float[] targetAngles, float velocity = -1f, float acceleration = -1f, float blendRadius = -1f, float time = -1f)
    {
        Debug.Log($"MoveJ: {targetAngles}, {velocity}, {acceleration}, {blendRadius}, {time}");     
        Goto(targetAngles, MovementType.MoveJ, velocity, acceleration, blendRadius, time);
    }
    
    public void MoveL(float[] targetPose, float velocity = -1f, float acceleration = -1f, float blendRadius = -1f, float time = -1f)
    {
        Debug.Log($"MoveL: {targetPose}, {velocity}, {acceleration}, {blendRadius}, {time}");
        Goto(targetPose, MovementType.MoveL, velocity, acceleration, blendRadius, time);
    }
}
