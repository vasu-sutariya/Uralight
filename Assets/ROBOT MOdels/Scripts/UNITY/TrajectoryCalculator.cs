using UnityEngine;
using System;

public enum TrajectoryType
{
    Cubic,
    Trapezoidal,
    Linear
}

public static class TrajectoryCalculator
{
    public static float CalculateCubicTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime)
    {
        return (float)(startAngle +
            ((3 / Math.Pow(totalTime, 2)) * shortestAngle * (currentTime * currentTime)) +
            ((-2 / Math.Pow(totalTime, 3)) * shortestAngle * Math.Pow(currentTime, 3)));
    }

    public static float CalculateRequiredTime(float distance, float maxVelocity, float maxAcceleration)
    {
        float timeFromVelocity = distance / maxVelocity;
        float timeFromAcceleration = Mathf.Sqrt(2 * distance / maxAcceleration);
        return Mathf.Max(timeFromVelocity, timeFromAcceleration);
    }

    // Trapezoidal trajectory time calculation
    public static float CalculateTrapezoidalTime(float distance, float maxVelocity, float maxAcceleration)
    {
        float dq = Mathf.Abs(distance);
        float direction = Mathf.Sign(distance);
        
        // Time to accelerate to vmax
        float t_acc = maxVelocity / maxAcceleration;
        float d_acc = 0.5f * maxAcceleration * t_acc * t_acc;
        
        float T;
        if (2 * d_acc >= dq)  // Triangular profile (can't reach vmax)
        {
            t_acc = Mathf.Sqrt(dq / maxAcceleration);
            T = 2 * t_acc;  // total time
        }
        else  // Trapezoidal profile
        {
            float d_flat = dq - 2 * d_acc;
            float t_flat = d_flat / maxVelocity;
            T = 2 * t_acc + t_flat;  // total time
        }
        
            
        return T;
    }

    // Centralized trajectory calculation function
    public static float CalculateTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime, 
        TrajectoryType trajectoryType = TrajectoryType.Cubic, bool timeGiven = false, bool blendEnabled = false, float maxVelocity = 30f, float maxAcceleration = 60f)
    {
        switch (trajectoryType)
        {
            case TrajectoryType.Cubic:
                return CalculateCubicTrajectory(startAngle, shortestAngle, currentTime, totalTime);
            
            case TrajectoryType.Trapezoidal:
                return CalculateTrapezoidalTrajectory(startAngle, shortestAngle, currentTime, totalTime, timeGiven, maxVelocity, maxAcceleration);
            
            case TrajectoryType.Linear:
                return CalculateLinearTrajectory(startAngle, shortestAngle, currentTime, totalTime);
            
            default:
                return CalculateCubicTrajectory(startAngle, shortestAngle, currentTime, totalTime);
        }
    }

    // Trapezoidal trajectory calculation
    public static float CalculateTrapezoidalTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime, bool timeGiven, float maxVelocity = 30f, float maxAcceleration = 60f)
    {
        // Filter out very small angles
        if (Mathf.Abs(startAngle) < 0.01f) startAngle = 0f;
        if (Mathf.Abs(shortestAngle) < 0.01f) shortestAngle = 0f;
        
        // If no movement needed, return start angle
        if (Mathf.Abs(shortestAngle) < 0.01f)
        {
            return startAngle;
        }
        
        Debug.Log($"CalculateTrapezoidalTrajectory: {startAngle}, {shortestAngle}, {currentTime}, {totalTime}, {timeGiven}, vel={maxVelocity}, acc={maxAcceleration}");
        float dq = Mathf.Abs(shortestAngle);
        float direction = Mathf.Sign(shortestAngle);
        
        // Time to accelerate to vmax
        float t_acc = maxVelocity / maxAcceleration;
        float d_acc = 0.5f * maxAcceleration * t_acc * t_acc;
        
        float t_flat = 0f;
        if (2 * d_acc >= dq)  // Triangular profile (can't reach vmax)
        {
            Debug.Log($"Triangular profile: dq={dq}, maxAcceleration={maxAcceleration}");
            t_acc = Mathf.Sqrt(dq / maxAcceleration);
            t_flat = 0f;
            Debug.Log($"Triangular profile: t_acc={t_acc}, t_flat={t_flat}");
        }
        else  // Trapezoidal profile
        {
            Debug.Log($"Trapezoidal profile: dq={dq}, maxVelocity={maxVelocity}");
            float d_flat = dq - 2 * d_acc;
            t_flat = d_flat / maxVelocity;
        }
        
        float qi;
        if (currentTime < t_acc)  // Acceleration phase
        {
            Debug.Log($"Acceleration phase: currentTime={currentTime}, t_acc={t_acc}");
            qi = startAngle + direction * 0.5f * maxAcceleration * currentTime * currentTime;
        }
        else if (currentTime < t_acc + t_flat)  // Constant velocity
        {
            Debug.Log($"Constant velocity phase: currentTime={currentTime}, t_acc={t_acc}, t_flat={t_flat}");
            qi = startAngle + direction * (d_acc + maxVelocity * (currentTime - t_acc));
        }
        else  // Deceleration phase
        {
            Debug.Log($"Deceleration phase: currentTime={currentTime}, t_acc={t_acc}, t_flat={t_flat}");
            float td = currentTime - (t_acc + t_flat);
            qi = startAngle + shortestAngle - direction * 0.5f * maxAcceleration * (t_acc - td) * (t_acc - td);
            Debug.Log($"Deceleration phase: qi={qi}, td={td}, t_acc={t_acc}, t_flat={t_flat}, startAngle={startAngle}, shortestAngle={shortestAngle}, direction={direction}, maxAcceleration={maxAcceleration}");
        }
        Debug.Log($"CalculateTrapezoidalTrajectory: qi={qi}");
        return qi;
    }

    // Placeholder for linear trajectory (to be implemented)
    public static float CalculateLinearTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime)
    {
        // TODO: Implement linear trajectory
        // For now, fall back to cubic
        return CalculateCubicTrajectory(startAngle, shortestAngle, currentTime, totalTime);
    }
} 