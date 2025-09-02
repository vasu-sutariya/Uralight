using UnityEngine;
using System;
using System.Collections.Generic;
public enum TrajectoryType
{
    Cubic,
    Trapezoidal,
    LinearCubic,
    LinearTrapezoidal,
}

public class TrajectoryCalculator: MonoBehaviour
{

    public URInverseKinematics inverseKinematics; // For MoveL calculations
    
    // Solution selection tracking
    private int previousSolutionIndex = -1; // -1 means no previous solution
    private bool isStarting = true; // Track if this is the first calculation

    // Smart solution selection method
    private float[] SelectBestSolution(List<float[]> solutions)
    {
        // If no solutions available, return zeros
        if (solutions == null || solutions.Count == 0)
        {
            Debug.LogWarning("No IK solutions found, returning zeros");
            
            return new float[] { 1000f, 0f, 0f, 0f, 0f, 0f };
        }

        int selectedIndex;

        if (isStarting)
        {
            // For starting: prefer 6th solution (index 5), if less than 6 solutions then choose 1st (index 0)
            if (solutions.Count >= 6)
            {
                selectedIndex = 5; // 6th solution
            }
            else
            {
                selectedIndex = 0; // 1st solution
            }
            isStarting = false; // Mark that we're no longer starting
        }
        else
        {
            // Later on: choose same as previous one if available, otherwise choose first available
            if (previousSolutionIndex >= 0 && previousSolutionIndex < solutions.Count)
            {
                selectedIndex = previousSolutionIndex;
            }
            else
            {
                selectedIndex = 0; // Fallback to first solution
            }
        }

        // Update the previous solution index for next time
        previousSolutionIndex = selectedIndex;

        Debug.Log($"Selected solution {selectedIndex + 1} out of {solutions.Count} available solutions");
        return solutions[selectedIndex];
    }

    // Method to reset solution selection state (useful for starting new trajectories)
    public void ResetSolutionSelection()
    {
        previousSolutionIndex = -1;
        isStarting = true;
        Debug.Log("Solution selection state reset - next calculation will be treated as starting");
    }

        // Centralized trajectory calculation function
    public float CalculateTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime, 
        TrajectoryType trajectoryType = TrajectoryType.Cubic, bool timeGiven = false, float maxVelocity = 30f, float maxAcceleration = 60f)
    {
        switch (trajectoryType)
        {
            case TrajectoryType.Cubic:
                return CalculateCubicTrajectory(startAngle, shortestAngle, currentTime, totalTime);
            case TrajectoryType.Trapezoidal:
                return CalculateTrapezoidalTrajectory(startAngle, shortestAngle, currentTime, totalTime, timeGiven, maxVelocity, maxAcceleration);
            default:
                return CalculateCubicTrajectory(startAngle, shortestAngle, currentTime, totalTime);
        }
    }

    public  float CalculateCubicTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime)
    {
        return (float)(startAngle +
            ((3 / Math.Pow(totalTime, 2)) * shortestAngle * (currentTime * currentTime)) +
            ((-2 / Math.Pow(totalTime, 3)) * shortestAngle * Math.Pow(currentTime, 3)));
    }

    // Trapezoidal trajectory time calculation
    public  float CalculateTrapezoidalTime(float distance, float maxVelocity, float maxAcceleration)
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



    // Trapezoidal trajectory calculation
    public  float CalculateTrapezoidalTrajectory(float startAngle, float shortestAngle, float currentTime, float totalTime, bool timeGiven, float maxVelocity = 30f, float maxAcceleration = 60f)
    {
        // Filter out very small angles
        if (Mathf.Abs(startAngle) < 0.01f) startAngle = 0f;
        if (Mathf.Abs(shortestAngle) < 0.01f) shortestAngle = 0f;
        
        // If no movement needed, return start angle
        if (Mathf.Abs(shortestAngle) < 0.01f)
        {
            return startAngle;
        }
        
        //Debug.Log($"CalculateTrapezoidalTrajectory: {startAngle}, {shortestAngle}, {currentTime}, {totalTime}, {timeGiven}, vel={maxVelocity}, acc={maxAcceleration}");
        float dq = Mathf.Abs(shortestAngle);
        float direction = Mathf.Sign(shortestAngle);
        
        // Time to accelerate to vmax
        float t_acc = maxVelocity / maxAcceleration;
        float d_acc = 0.5f * maxAcceleration * t_acc * t_acc;
        
        float t_flat = 0f;
        if (2 * d_acc >= dq)  // Triangular profile (can't reach vmax)
        {
            //Debug.Log($"Triangular profile: dq={dq}, maxAcceleration={maxAcceleration}");
            t_acc = Mathf.Sqrt(dq / maxAcceleration);
            t_flat = 0f;
            //Debug.Log($"Triangular profile: t_acc={t_acc}, t_flat={t_flat}");
        }
        else  // Trapezoidal profile
        {
            //Debug.Log($"Trapezoidal profile: dq={dq}, maxVelocity={maxVelocity}");
            float d_flat = dq - 2 * d_acc;
            t_flat = d_flat / maxVelocity;
        }
        
        float qi;
        if (currentTime < t_acc)  // Acceleration phase
        {
            //Debug.Log($"Acceleration phase: currentTime={currentTime}, t_acc={t_acc}");
            qi = startAngle + direction * 0.5f * maxAcceleration * currentTime * currentTime;
        }
        else if (currentTime < t_acc + t_flat)  // Constant velocity
        {
            //Debug.Log($"Constant velocity phase: currentTime={currentTime}, t_acc={t_acc}, t_flat={t_flat}");
            qi = startAngle + direction * (d_acc + maxVelocity * (currentTime - t_acc));
        }
        else  // Deceleration phase
        {
            //Debug.Log($"Deceleration phase: currentTime={currentTime}, t_acc={t_acc}, t_flat={t_flat}");
            float td = currentTime - (t_acc + t_flat);
            qi = startAngle + shortestAngle - direction * 0.5f * maxAcceleration * (t_acc - td) * (t_acc - td);
            //Debug.Log($"Deceleration phase: qi={qi}, td={td}, t_acc={t_acc}, t_flat={t_flat}, startAngle={startAngle}, shortestAngle={shortestAngle}, direction={direction}, maxAcceleration={maxAcceleration}");
        }
        //Debug.Log($"CalculateTrapezoidalTrajectory: qi={qi}");
        return qi;
    }

   public  float[] CalculateLinearTrajectory( Matrix4x4 startMatrix, Matrix4x4 endMatrix, Vector3 u, float theta, float currentTime, float totalTime, 
        TrajectoryType trajectoryType = TrajectoryType.LinearCubic, bool timeGiven = false, float maxVelocity = 30f, float maxAcceleration = 60f)
    {
        switch (trajectoryType)
        {
            case TrajectoryType.LinearCubic:
                return CalculateLinearCubicTrajectory(startMatrix, endMatrix, u, theta, currentTime, totalTime);
            
            case TrajectoryType.LinearTrapezoidal:
                return CalculateLinearCubicTrajectory(startMatrix, endMatrix, u, theta, currentTime, totalTime);
            
            default:
                return CalculateLinearCubicTrajectory(startMatrix, endMatrix, u, theta, currentTime, totalTime);
        }
    }
    
    public float[] CalculateLinearCubicTrajectory(Matrix4x4 startMatrix, Matrix4x4 endMatrix, Vector3 u, float theta, float currentTime, float totalTime)
    {

        var tau = currentTime / totalTime;
        var cubicScale = (3 * tau * tau) - (2 * tau * tau * tau);
        Debug.Log($"cubicScale = {cubicScale}");
        Debug.Log($"theta = {theta}");
        Vector3 currentaxis = new Vector3(u.x, u.y, u.z);
        float currentangle = cubicScale* theta * Mathf.Rad2Deg;
        
        Vector3 currentrpy = AXisangle.AxisAngleToRPYDegrees(currentaxis, currentangle);

        float x = (startMatrix[0 , 3] +
            ((3 / Mathf.Pow(totalTime, 2)) * (endMatrix[0 , 3] - startMatrix[0 , 3]) * (currentTime * currentTime)) +
            ((-2 / Mathf.Pow(totalTime, 3)) * (endMatrix[0 , 3] - startMatrix[0 , 3]) * Mathf.Pow(currentTime, 3)));
        
        float y = (startMatrix[1 , 3] +
            ((3 / Mathf.Pow(totalTime, 2)) * (endMatrix[1 , 3] - startMatrix[1 , 3]) * (currentTime * currentTime)) +
            ((-2 / Mathf.Pow(totalTime, 3)) * (endMatrix[1 , 3] - startMatrix[1 , 3]) * Mathf.Pow(currentTime, 3)));
        
        float z = (startMatrix[2 , 3] +
            ((3 / Mathf.Pow(totalTime, 2)) * (endMatrix[2 , 3] - startMatrix[2 , 3]) * (currentTime * currentTime)) +
            ((-2 / Mathf.Pow(totalTime, 3)) * (endMatrix[2 , 3] - startMatrix[2 , 3]) * Mathf.Pow(currentTime, 3)));

        Debug.Log($"CalculateLinearCubicTrajectory: x={x}, y={y}, z={z}, u = {u.x}, {u.y}, {u.z}, currentrpy={currentrpy.x}, {currentrpy.y}, {currentrpy.z}, currentangle={currentangle}");
        Matrix4x4 currentMatrix = inverseKinematics.PoseToTransform(x, y, z, currentrpy.x, currentrpy.y, currentrpy.z);
        List<float[]> solutions = inverseKinematics.CalculateIK(currentMatrix);

        float[] currentAngles = SelectBestSolution(solutions);



        return new float[] {currentAngles[0], currentAngles[1], currentAngles[2], currentAngles[3], currentAngles[4], currentAngles[5], currentangle};
    }


   
} 