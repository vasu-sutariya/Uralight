using System;
using UnityEngine;

/// <summary>
/// Utilities for converting between Roll-Pitch-Yaw (RPY), axis-angle and Quaternion.
/// Convention:
/// - roll about X, pitch about Y, yaw about Z (RPY = [roll_x, pitch_y, yaw_z])
/// - composition order: yaw · pitch · roll (i.e., q = qYaw * qPitch * qRoll)
/// - degrees variants use degrees; radians variants use radians
/// </summary>
public static class AXisangle
{
	// -----------------------------
	// Public: Degrees-based helpers
	// -----------------------------

	/// <summary>
	/// Convert RPY degrees (roll_x, pitch_y, yaw_z) to axis-angle.
	/// </summary>
  
    public static void MatrixToAxisAngle(Matrix4x4 matrix, out Vector3 axis, out float angle)
    {
        angle = (float)Math.Acos((matrix[0,0] + matrix[1,1] + matrix[2,2] - 1) / 2.0);
        if (Math.Abs(angle) < 1e-6)
        {
            axis = new Vector3(1, 0, 0);
            angle = 0;
            return;
        }

        axis = new Vector3(
            (float)((matrix[2,1] - matrix[1,2]) / (2 * Math.Sin(angle))),
            (float)((matrix[0,2] - matrix[2,0]) / (2 * Math.Sin(angle))),
            (float)((matrix[1,0] - matrix[0,1]) / (2 * Math.Sin(angle)))
        );
    }

    public static void MatrixToRPY(Matrix4x4 T, out Vector3 rpy)
    {
        float roll = Mathf.Atan2(T[2, 1], T[2, 2]) * Mathf.Rad2Deg;
        float pitch = Mathf.Atan2(-T[2, 0], Mathf.Sqrt(T[2, 1] * T[2, 1] + T[2, 2] * T[2, 2])) * Mathf.Rad2Deg;
        float yaw = Mathf.Atan2(T[1, 0], T[0, 0]) * Mathf.Rad2Deg;
        rpy = new Vector3(roll, pitch, yaw);
    }
    

	public static void RPYDegreesToAxisAngle(Vector3 rpyDegrees, out Vector3 axis, out float angleDegrees)
	{
		Quaternion q = RPYDegreesToQuaternion(rpyDegrees);
		QuaternionToAxisAngle(q, out axis, out angleDegrees);
	}

	/// <summary>
	/// Convert axis-angle (degrees) to RPY degrees (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Vector3 AxisAngleToRPYDegrees(Vector3 axis, float angleDegrees)
	{
		Quaternion q = AxisAngleToQuaternion(axis, angleDegrees);
		return QuaternionToRPYDegrees(q);
	}

	/// <summary>
	/// Build Quaternion from RPY degrees (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Quaternion RPYDegreesToQuaternion(Vector3 rpyDegrees)
	{
		Vector3 rpyRad = rpyDegrees * Mathf.Deg2Rad;
		return RPYRadiansToQuaternion(rpyRad);
	}

	/// <summary>
	/// Convert Quaternion to RPY degrees (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Vector3 QuaternionToRPYDegrees(Quaternion q)
	{
		Vector3 rpyRad = QuaternionToRPYRadians(q);
		return rpyRad * Mathf.Rad2Deg;
	}

	/// <summary>
	/// Convert Quaternion to axis-angle (angle in degrees).
	/// </summary>
	public static void QuaternionToAxisAngle(Quaternion q, out Vector3 axis, out float angleDegrees)
	{
		if (q.w > 1f || q.w < -1f)
		{
			q = NormalizeQuaternion(q);
		}
		q.ToAngleAxis(out angleDegrees, out axis);
		if (axis.sqrMagnitude < 1e-12f)
		{
			axis = Vector3.right;
			angleDegrees = 0f;
		}
	}

	/// <summary>
	/// Convert axis-angle (angle in degrees) to Quaternion.
	/// </summary>
	public static Quaternion AxisAngleToQuaternion(Vector3 axis, float angleDegrees)
	{
		Vector3 n = axis.sqrMagnitude > 0f ? axis.normalized : Vector3.right;
		return Quaternion.AngleAxis(angleDegrees, n);
	}

	// -----------------------------
	// Public: Rotation Vector helpers (RX, RY, RZ)
	// -----------------------------

	/// <summary>
	/// Convert RPY degrees to rotation vector (RX, RY, RZ) in degrees.
	/// Rotation vector = axis * angle, where magnitude is angle in degrees.
	/// </summary>
	public static Vector3 RPYDegreesToRotationVector(Vector3 rpyDegrees)
	{
		RPYDegreesToAxisAngle(rpyDegrees, out Vector3 axis, out float angleDegrees);
		return axis * angleDegrees;
	}

	/// <summary>
	/// Convert rotation vector (RX, RY, RZ) in degrees to RPY degrees.
	/// </summary>
	public static Vector3 RotationVectorToRPYDegrees(Vector3 rotationVector)
	{
		float angleDegrees = rotationVector.magnitude;
		Vector3 axis = angleDegrees > 1e-6f ? rotationVector.normalized : Vector3.right;
		return AxisAngleToRPYDegrees(axis, angleDegrees);
	}

	/// <summary>
	/// Convert RPY radians to rotation vector (RX, RY, RZ) in radians.
	/// Rotation vector = axis * angle, where magnitude is angle in radians.
	/// </summary>
	public static Vector3 RPYRadiansToRotationVector(Vector3 rpyRadians)
	{
		RPYRadiansToAxisAngle(rpyRadians, out Vector3 axis, out float angleRadians);
		return axis * angleRadians;
	}

	/// <summary>
	/// Convert rotation vector (RX, RY, RZ) in radians to RPY radians.
	/// </summary>
	public static Vector3 RotationVectorToRPYRadians(Vector3 rotationVector)
	{
		float angleRadians = rotationVector.magnitude;
		Vector3 axis = angleRadians > 1e-6f ? rotationVector.normalized : Vector3.right;
		return AxisAngleToRPYRadians(axis, angleRadians);
	}

	/// <summary>
	/// Convert Quaternion to rotation vector (RX, RY, RZ) in degrees.
	/// </summary>
	public static Vector3 QuaternionToRotationVector(Quaternion q)
	{
		QuaternionToAxisAngle(q, out Vector3 axis, out float angleDegrees);
		return axis * angleDegrees;
	}

	/// <summary>
	/// Convert rotation vector (RX, RY, RZ) in degrees to Quaternion.
	/// </summary>
	public static Quaternion RotationVectorToQuaternion(Vector3 rotationVector)
	{
		float angleDegrees = rotationVector.magnitude;
		Vector3 axis = angleDegrees > 1e-6f ? rotationVector.normalized : Vector3.right;
		return AxisAngleToQuaternion(axis, angleDegrees);
	}

	/// <summary>
	/// Convert rotation vector (RX, RY, RZ) in radians to Quaternion.
	/// </summary>
	public static Quaternion RotationVectorRadiansToQuaternion(Vector3 rotationVector)
	{
		float angleRadians = rotationVector.magnitude;
		Vector3 axis = angleRadians > 1e-6f ? rotationVector.normalized : Vector3.right;
		return AxisAngleToQuaternion(axis, angleRadians * Mathf.Rad2Deg);
	}

	/// <summary>
	/// Convert Quaternion to rotation vector (RX, RY, RZ) in radians.
	/// </summary>
	public static Vector3 QuaternionToRotationVectorRadians(Quaternion q)
	{
		QuaternionToAxisAngle(q, out Vector3 axis, out float angleDegrees);
		return axis * (angleDegrees * Mathf.Deg2Rad);
	}

	// -----------------------------
	// Public: Radians-based helpers
	// -----------------------------

	/// <summary>
	/// Convert RPY radians (roll_x, pitch_y, yaw_z) to axis-angle (radians).
	/// </summary>
	public static void RPYRadiansToAxisAngle(Vector3 rpyRadians, out Vector3 axis, out float angleRadians)
	{
		Quaternion q = RPYRadiansToQuaternion(rpyRadians);
		QuaternionToAxisAngle(q, out axis, out float angleDeg);
		angleRadians = angleDeg * Mathf.Deg2Rad;
	}

	/// <summary>
	/// Convert axis-angle (radians) to RPY radians (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Vector3 AxisAngleToRPYRadians(Vector3 axis, float angleRadians)
	{
		Quaternion q = AxisAngleToQuaternion(axis, angleRadians * Mathf.Rad2Deg);
		return QuaternionToRPYRadians(q);
	}

	/// <summary>
	/// Build Quaternion from RPY radians (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Quaternion RPYRadiansToQuaternion(Vector3 rpyRadians)
	{
		float halfRoll = 0.5f * rpyRadians.x;
		float halfPitch = 0.5f * rpyRadians.y;
		float halfYaw = 0.5f * rpyRadians.z;

		float cRoll = Mathf.Cos(halfRoll);
		float sRoll = Mathf.Sin(halfRoll);
		float cPitch = Mathf.Cos(halfPitch);
		float sPitch = Mathf.Sin(halfPitch);
		float cYaw = Mathf.Cos(halfYaw);
		float sYaw = Mathf.Sin(halfYaw);

		// q = qYaw * qPitch * qRoll (intrinsic rotations: roll X, then pitch Y, then yaw Z)
		float w = cRoll * cPitch * cYaw + sRoll * sPitch * sYaw;
		float x = sRoll * cPitch * cYaw - cRoll * sPitch * sYaw;
		float y = cRoll * sPitch * cYaw + sRoll * cPitch * sYaw;
		float z = cRoll * cPitch * sYaw - sRoll * sPitch * cYaw;

		Quaternion q = new Quaternion(x, y, z, w);
		return NormalizeQuaternion(q);
	}

	/// <summary>
	/// Convert Quaternion to RPY radians (roll_x, pitch_y, yaw_z).
	/// </summary>
	public static Vector3 QuaternionToRPYRadians(Quaternion q)
	{
		q = NormalizeQuaternion(q);

		float w = q.w;
		float x = q.x;
		float y = q.y;
		float z = q.z;

		// roll (x-axis rotation)
		float sinr_cosp = 2f * (w * x + y * z);
		float cosr_cosp = 1f - 2f * (x * x + y * y);
		float roll = Mathf.Atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		float sinp = 2f * (w * y - z * x);
		float pitch;
		if (Mathf.Abs(sinp) >= 1f)
		{
			pitch = Mathf.Sign(sinp) * Mathf.PI / 2f;
		}
		else
		{
			pitch = Mathf.Asin(sinp);
		}

		// yaw (z-axis rotation)
		float siny_cosp = 2f * (w * z + x * y);
		float cosy_cosp = 1f - 2f * (y * y + z * z);
		float yaw = Mathf.Atan2(siny_cosp, cosy_cosp);

		return new Vector3(roll, pitch, yaw);
	}

	// -----------------------------
	// Internal helpers
	// -----------------------------

	private static Quaternion NormalizeQuaternion(Quaternion q)
	{
		float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
		if (mag <= 0f)
		{
			return Quaternion.identity;
		}
		float inv = 1f / mag;
		return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
	}

    
}


