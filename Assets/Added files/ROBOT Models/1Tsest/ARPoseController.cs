using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public enum SensorFusionType
{
    Complementary,
    Kalman
}

public class ARPoseController : MonoBehaviour
{
    // Simple 1D Kalman Filter for individual sensor axes
    [System.Serializable]
    private class KalmanFilter
    {
        private float x; // State estimate
        private float P; // Estimate error covariance
        private float Q; // Process noise covariance
        private float R; // Measurement noise covariance
        
        public KalmanFilter(float processNoise = 0.01f, float measurementNoise = 0.1f, float initialEstimateError = 1.0f)
        {
            Q = processNoise;
            R = measurementNoise;
            P = initialEstimateError;
            x = 0f;
        }
        
        public float Update(float measurement, float deltaTime = 1f)
        {
            // Prediction step
            // x = x (no change in state prediction for this simple model)
            P = P + Q * deltaTime; // Update estimate error covariance
            
            // Update step
            float K = P / (P + R); // Kalman gain
            x = x + K * (measurement - x); // Update estimate
            P = (1 - K) * P; // Update estimate error covariance
            
            return x;
        }
        
        public float GetState() => x;
        public void SetState(float state) => x = state;
        public void Reset(float initialValue = 0f) 
        { 
            x = initialValue; 
            P = 1.0f; 
        }
    }

    // 3D Kalman Filter for Vector3 data
    [System.Serializable]
    private class KalmanFilter3D
    {
        private KalmanFilter filterX;
        private KalmanFilter filterY;
        private KalmanFilter filterZ;
        
        public KalmanFilter3D(float processNoise = 0.01f, float measurementNoise = 0.1f, float initialEstimateError = 1.0f)
        {
            filterX = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
            filterY = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
            filterZ = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
        }
        
        public Vector3 Update(Vector3 measurement, float deltaTime = 1f)
        {
            return new Vector3(
                filterX.Update(measurement.x, deltaTime),
                filterY.Update(measurement.y, deltaTime),
                filterZ.Update(measurement.z, deltaTime)
            );
        }
        
        public Vector3 GetState()
        {
            return new Vector3(filterX.GetState(), filterY.GetState(), filterZ.GetState());
        }
        
        public void SetState(Vector3 state)
        {
            filterX.SetState(state.x);
            filterY.SetState(state.y);
            filterZ.SetState(state.z);
        }
        
        public void Reset(Vector3 initialValue = default)
        {
            filterX.Reset(initialValue.x);
            filterY.Reset(initialValue.y);
            filterZ.Reset(initialValue.z);
        }
        
        public void UpdateParameters(float processNoise, float measurementNoise, float initialEstimateError)
        {
            filterX = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
            filterY = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
            filterZ = new KalmanFilter(processNoise, measurementNoise, initialEstimateError);
        }
    }

    [Header("Network Settings")]
    [SerializeField] private int udpPort = 8080;
    
    [Header("Pose Control Settings")]
    [SerializeField] private float rotationSensitivity = 1.0f;
    [SerializeField] private bool invertY = true;
    [SerializeField] private bool useRelativeMovement = true;
    [SerializeField] private bool reverseRotations = false; // Reverse all rotations
    
    [Header("Position Sensitivity")]
    [SerializeField] public float positionSensitivityX = 1.0f;
    [SerializeField] public float positionSensitivityY = 1.0f;
    [SerializeField] public float positionSensitivityZ = 1.0f;
    
    [Header("Position Offsets")]
    [SerializeField] public float positionOffsetX = 0.0f;
    [SerializeField] public float positionOffsetY = 0.0f;
    [SerializeField] public float positionOffsetZ = 0.0f;
    
    [Header("Axis Mapping")]
    [SerializeField] public AxisMapping inputToRobotMapping = AxisMapping.Direct;
    
    public enum AxisMapping
    {
        Direct,         // X→X, Y→Y, Z→Z
        SwapXY,         // X→Y, Y→X, Z→Z
        SwapXZ,         // X→Z, Y→Y, Z→X
        SwapYZ,         // X→X, Y→Z, Z→Y
        SwapXYAndXZ,    // X→Z, Y→X, Z→Y
        SwapXYAndYZ,    // X→Y, Y→Z, Z→X
        SwapXZAndYZ,    // X→X, Y→Z, Z→Y
        Custom          // Custom mapping defined below
    }
    
    [Header("Custom Axis Mapping")]
    [SerializeField] public RobotAxis inputXToRobotAxis = RobotAxis.X;
    [SerializeField] public RobotAxis inputYToRobotAxis = RobotAxis.Y;
    [SerializeField] public RobotAxis inputZToRobotAxis = RobotAxis.Z;
    
    public enum RobotAxis
    {
        X,
        Y,
        Z
    }
    
    [Header("Sensor Fusion Settings")]
    [SerializeField] private bool useSensorFusion = true;
    [SerializeField] private SensorFusionType fusionType = SensorFusionType.Kalman;
    
    [Header("Complementary Filter Settings")]
    [SerializeField] private float gyroWeight = 0.95f; // Complementary filter weight for gyro (reduced for less drift)
    [SerializeField] private float accWeight = 0.05f;  // Complementary filter weight for accelerometer (increased for stability)
    [SerializeField] private float smoothingFactor = 0.1f; // Low-pass filter for smoothing
    [SerializeField] private bool useGyroIntegration = true;
    [SerializeField] private float gyroDriftCorrection = 0.01f; // Increased drift correction
    
    [Header("Drift Correction Settings")]
    [SerializeField] private bool enableDriftCorrection = true;
    [SerializeField] private float driftCorrectionStrength = 0.02f; // How aggressively to correct drift
    [SerializeField] private float gyroDeadZone = 0.1f; // Ignore small gyro movements (rad/s)
    [SerializeField] private bool resetDriftOnCalibrate = true;
    
    [Header("Kalman Filter Settings")]
    [SerializeField] private float processNoise = 0.01f; // Q - Process noise covariance
    [SerializeField] private float measurementNoise = 0.1f; // R - Measurement noise covariance
    [SerializeField] private float initialEstimateError = 1.0f; // P - Initial estimate error covariance
    
    [Header("AR Pose vs IMU Trust Settings")]
    [SerializeField] private float arPoseWeight = 0.8f; // How much to trust AR pose data (0.0 to 1.0)
    [SerializeField] private float imuWeight = 0.2f; // How much to trust IMU data (0.0 to 1.0)
    [SerializeField] private float arPoseMeasurementNoise = 0.05f; // Lower = more trust in AR pose
    [SerializeField] private float imuMeasurementNoise = 0.3f; // Higher = less trust in IMU
    
    [Header("Calibration")]
    [SerializeField] private KeyCode calibrateKey = KeyCode.Space;
    [SerializeField] private bool autoCalibrate = true;
    
    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = true;
    [SerializeField] private bool showSensorData = false;
    
    [Header("Robot Control")]
    [SerializeField] private GameObject robotObject;
    [SerializeField] private URInverseKinematics inverseKinematics;
    [SerializeField] private UnityJointController jointController;
    
    // Network components
    private UdpClient udpClient;
    private Thread udpThread;
    private bool isReceiving = false;
    
    // Pose data
    private Vector3 currentPosition;
    private Vector3 currentRotation;
    private Vector3 centerPosition;
    private Vector3 centerRotation;
    private bool isCalibrated = false;
    
    // Sensor data
    private Vector3 gyroscopeData;
    private Vector3 accelerometerData;
    private Vector3 magnetometerData; // Optional for full 9DOF
    
    // Sensor fusion variables
    private Vector3 fusedRotation;
    private Vector3 previousRotation;
    private Vector3 gyroIntegratedRotation;
    private Vector3 smoothedPosition;
    private Vector3 smoothedRotation;
    private float lastUpdateTime;
    
    // Drift correction variables
    private Vector3 gyroBias = Vector3.zero; // Estimated gyro bias
    private Vector3 gyroCalibrationSum = Vector3.zero;
    private int gyroCalibrationCount = 0;
    private bool isGyroCalibrated = false;
    private float lastDriftCorrectionTime = 0f;
    
    // Kalman filter instances
    private KalmanFilter3D positionKalman;
    private KalmanFilter3D rotationKalman;
    private KalmanFilter3D gyroKalman;
    private KalmanFilter3D accelKalman;
    
    // Thread-safe data exchange
    private readonly object dataLock = new object();
    private bool hasNewData = false;
    private float[] latestSensorData = new float[12]; // x,y,z,rx,ry,rz,gx,gy,gz,ax,ay,az
    
    void Start()
    {
        StartUDPListener();
        lastUpdateTime = Time.time;
        
        // Initialize Kalman filters
        InitializeKalmanFilters();
        
        // Find robot components if not assigned
        if (robotObject == null)
            robotObject = GameObject.Find("UR10");
        if (inverseKinematics == null && robotObject != null)
            inverseKinematics = robotObject.GetComponentInChildren<URInverseKinematics>();
        if (jointController == null && robotObject != null)
            jointController = robotObject.GetComponentInChildren<UnityJointController>();
        
        if (showDebugInfo)
        {
            Debug.Log("AR Pose Controller with Sensor Fusion Ready!");
            Debug.Log($"Listening on UDP port {udpPort}");
            Debug.Log($"Press {calibrateKey} to calibrate center position");
            Debug.Log($"Sensor Fusion: {(useSensorFusion ? "Enabled" : "Disabled")} - Type: {fusionType}");
        }
    }
    
    void InitializeKalmanFilters()
    {
        // AR pose data gets lower noise (more trusted)
        positionKalman = new KalmanFilter3D(processNoise, arPoseMeasurementNoise, initialEstimateError);
        rotationKalman = new KalmanFilter3D(processNoise, arPoseMeasurementNoise, initialEstimateError);
        
        // IMU data gets higher noise (less trusted)
        gyroKalman = new KalmanFilter3D(processNoise * 0.1f, imuMeasurementNoise, initialEstimateError);
        accelKalman = new KalmanFilter3D(processNoise * 2f, imuMeasurementNoise * 1.5f, initialEstimateError);
    }
    
    void Update()
    {
        // Handle calibration input
        if (Input.GetKeyDown(calibrateKey))
        {
            CalibrateCenter();
        }
        
        // Update Kalman filter parameters if they changed
        UpdateKalmanParameters();
        
        // Apply pose data to GameObject
        ApplyPoseData();
    }
    
    void UpdateKalmanParameters()
    {
        if (fusionType == SensorFusionType.Kalman && positionKalman != null)
        {
            // Normalize weights to ensure they add up to 1.0
            float totalWeight = arPoseWeight + imuWeight;
            if (totalWeight > 0)
            {
                arPoseWeight = arPoseWeight / totalWeight;
                imuWeight = imuWeight / totalWeight;
            }
            
            // Update AR pose filters (more trusted)
            positionKalman.UpdateParameters(processNoise, arPoseMeasurementNoise, initialEstimateError);
            rotationKalman.UpdateParameters(processNoise, arPoseMeasurementNoise, initialEstimateError);
            
            // Update IMU filters (less trusted)
            gyroKalman.UpdateParameters(processNoise * 0.1f, imuMeasurementNoise, initialEstimateError);
            accelKalman.UpdateParameters(processNoise * 2f, imuMeasurementNoise * 1.5f, initialEstimateError);
        }
    }
    
    void StartUDPListener()
    {
        try
        {
            udpClient = new UdpClient(udpPort);
            isReceiving = true;
            
            udpThread = new Thread(new ThreadStart(ReceiveUDPData));
            udpThread.IsBackground = true;
            udpThread.Start();
            
            if (showDebugInfo)
                Debug.Log($"UDP listener started on port {udpPort}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to start UDP listener: {e.Message}");
        }
    }
    
    void ReceiveUDPData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        
        while (isReceiving)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                //Debug.Log("Received data: " + data);
                string message = Encoding.UTF8.GetString(data);

                //Debug.Log("Received message: " + message);
                // Parse CSV format: x,y,z,rx,ry,rz,gx,gy,gz,ax,ay,az
                string[] values = message.Split(',');
                if (values.Length >= 6) // Support both old format (6 values) and new format (12 values)
                {
                    lock (dataLock)
                    {
                        int maxValues = Mathf.Min(values.Length, latestSensorData.Length);
                        for (int i = 0; i < maxValues; i++)
                        {
                            if (float.TryParse(values[i], out float value))
                            {
                                latestSensorData[i] = value;
                            }
                        }
                        hasNewData = true;
                    }
                }
            }
            catch (System.Exception e)
            {
                if (isReceiving)
                    Debug.LogError($"UDP receive error: {e.Message}");
            }
        }
    }
    
    void ApplyPoseData()
    {
        lock (dataLock)
        {
            if (!hasNewData) return;
            
            // Extract all sensor data
            float x = latestSensorData[0];
            float y = latestSensorData[1];
            float z = latestSensorData[2];
            float rx = latestSensorData[3];
            float ry = latestSensorData[4];
            float rz = latestSensorData[5];
            
            // Extract gyro and accelerometer data (if available)
            Vector3 gyro = Vector3.zero;
            Vector3 accel = Vector3.zero;
            
            if (latestSensorData.Length >= 12)
            {
                gyro = new Vector3(latestSensorData[6], latestSensorData[7], latestSensorData[8]);
                accel = new Vector3(latestSensorData[9], latestSensorData[10], latestSensorData[11]);
            }
            
            currentPosition = new Vector3(x, y, z);
            currentRotation = new Vector3(rx, ry, rz);
            gyroscopeData = gyro;
            accelerometerData = accel;
            
            // Apply sensor fusion if enabled
            if (useSensorFusion && latestSensorData.Length >= 12)
            {
                if (fusionType == SensorFusionType.Kalman)
                {
                    ApplyKalmanFusion();
                }
                else
                {
                    ApplySensorFusion(); // Complementary filter
                }
            }
            else
            {
                // Use basic pose data without fusion
                fusedRotation = currentRotation;
            }
            
            // Apply smoothing
            ApplySmoothing();
            
            // Auto-calibrate on first data
            if (autoCalibrate && !isCalibrated)
            {
                CalibrateCenter();
            }
            
            if (isCalibrated)
            {
                ApplyPoseToGameObject();
            }
            
            // Debug output
            if (showDebugInfo)
            {
                Debug.Log($"Pose: Pos({x:F2}, {y:F2}, {z:F2}) Rot({fusedRotation.x:F1}°, {fusedRotation.y:F1}°, {fusedRotation.z:F1}°)");
            }
            
            if (showSensorData && latestSensorData.Length >= 12)
            {
                Debug.Log($"Gyro({gyro.x:F2}, {gyro.y:F2}, {gyro.z:F2}) Accel({accel.x:F2}, {accel.y:F2}, {accel.z:F2})");
            }
            
            hasNewData = false;
        }
    }
    
    void ApplySensorFusion()
    {
        float deltaTime = Time.time - lastUpdateTime;
        lastUpdateTime = Time.time;
        
        if (deltaTime <= 0) return; // Avoid division by zero
        
        // POSITION: Keep using AR pose data as-is (DO NOT CHANGE)
        // currentPosition is already set from AR pose data
        
        // ROTATION: Use ONLY accelerometer and gyroscope fusion (ignore AR pose rotation)
        
        // Apply gyro bias correction and dead zone
        Vector3 correctedGyro = gyroscopeData - gyroBias;
        
        // Apply dead zone to reduce drift from sensor noise
        if (Mathf.Abs(correctedGyro.x) < gyroDeadZone) correctedGyro.x = 0f;
        if (Mathf.Abs(correctedGyro.y) < gyroDeadZone) correctedGyro.y = 0f;
        if (Mathf.Abs(correctedGyro.z) < gyroDeadZone) correctedGyro.z = 0f;
        
        // Calculate rotation from accelerometer (tilt sensing)
        Vector3 accelRotation = CalculateRotationFromAccelerometer(accelerometerData);
        
        // Gyroscope integration for high-frequency rotation tracking
        if (useGyroIntegration)
        {
            // Convert corrected gyro from rad/s to degrees and integrate
            Vector3 gyroRotation = correctedGyro * Mathf.Rad2Deg * deltaTime;
            gyroIntegratedRotation += gyroRotation;
            
            // Enhanced drift correction
            if (enableDriftCorrection)
            {
                // Stronger correction for pitch and roll using accelerometer
                gyroIntegratedRotation.x = Mathf.Lerp(gyroIntegratedRotation.x, accelRotation.x, driftCorrectionStrength);
                gyroIntegratedRotation.z = Mathf.Lerp(gyroIntegratedRotation.z, accelRotation.z, driftCorrectionStrength);
                
                // Lighter correction for yaw (since accelerometer can't measure yaw directly)
                // Slowly drift yaw towards zero if no significant rotation detected
                if (Mathf.Abs(correctedGyro.y) < gyroDeadZone * 0.5f)
                {
                    gyroIntegratedRotation.y = Mathf.Lerp(gyroIntegratedRotation.y, 0f, driftCorrectionStrength * 0.1f);
                }
            }
        }
        
        // Complementary filter: Combine gyro (high frequency) with accelerometer (low frequency)
        // Use ONLY IMU data - no AR pose rotation
        if (useGyroIntegration)
        {
            fusedRotation.x = gyroWeight * gyroIntegratedRotation.x + accWeight * accelRotation.x;
            fusedRotation.z = gyroWeight * gyroIntegratedRotation.z + accWeight * accelRotation.z;
            // Y rotation (yaw) from gyro integration only
            fusedRotation.y = gyroIntegratedRotation.y;
        }
        else
        {
            // Use accelerometer for pitch/roll, gyro for yaw
            fusedRotation.x = accelRotation.x; // Pitch from accelerometer
            fusedRotation.z = accelRotation.z; // Roll from accelerometer
            fusedRotation.y = correctedGyro.y * Mathf.Rad2Deg * deltaTime; // Yaw from corrected gyro
        }
        
        // Clamp rotation values to reasonable ranges
        fusedRotation.x = Mathf.Clamp(fusedRotation.x, -90f, 90f);  // Pitch
        fusedRotation.z = Mathf.Clamp(fusedRotation.z, -90f, 90f);  // Roll
        
        previousRotation = fusedRotation;
    }
    
    void ApplyKalmanFusion()
    {
        float deltaTime = Time.time - lastUpdateTime;
        lastUpdateTime = Time.time;
        
        if (deltaTime <= 0) return; // Avoid division by zero
        
        // Apply gyro bias correction and dead zone
        Vector3 correctedGyro = gyroscopeData - gyroBias;
        
        // Apply dead zone to reduce drift from sensor noise
        if (Mathf.Abs(correctedGyro.x) < gyroDeadZone) correctedGyro.x = 0f;
        if (Mathf.Abs(correctedGyro.y) < gyroDeadZone) correctedGyro.y = 0f;
        if (Mathf.Abs(correctedGyro.z) < gyroDeadZone) correctedGyro.z = 0f;
        
        // Filter corrected sensor data through Kalman filters
        Vector3 filteredGyro = gyroKalman.Update(correctedGyro, deltaTime);
        Vector3 filteredAccel = accelKalman.Update(accelerometerData, deltaTime);
        
        // POSITION: Keep using AR pose data as-is (DO NOT CHANGE)
        // No filtering applied to position - use currentPosition directly
        
        // ROTATION: Use ONLY accelerometer and gyroscope fusion (ignore AR pose rotation)
        Vector3 accelRotation = CalculateRotationFromAccelerometer(filteredAccel);
        
        // Gyroscope integration with filtered data
        if (useGyroIntegration)
        {
            // Convert filtered gyro from rad/s to degrees and integrate
            Vector3 gyroRotation = filteredGyro * Mathf.Rad2Deg * deltaTime;
            gyroIntegratedRotation += gyroRotation;
            
            // Enhanced drift correction
            if (enableDriftCorrection)
            {
                // Stronger correction for pitch and roll using accelerometer
                gyroIntegratedRotation.x = Mathf.Lerp(gyroIntegratedRotation.x, accelRotation.x, driftCorrectionStrength);
                gyroIntegratedRotation.z = Mathf.Lerp(gyroIntegratedRotation.z, accelRotation.z, driftCorrectionStrength);
                
                // Lighter correction for yaw
                if (Mathf.Abs(filteredGyro.y) < gyroDeadZone * 0.5f)
                {
                    gyroIntegratedRotation.y = Mathf.Lerp(gyroIntegratedRotation.y, 0f, driftCorrectionStrength * 0.1f);
                }
            }
        }
        
        // Create rotation using ONLY IMU data (gyro + accelerometer)
        Vector3 imuOnlyRotation;
        if (useGyroIntegration)
        {
            // Combine gyro integration with accelerometer
            imuOnlyRotation = new Vector3(
                gyroWeight * gyroIntegratedRotation.x + accWeight * accelRotation.x,
                gyroIntegratedRotation.y, // Yaw from gyro only
                gyroWeight * gyroIntegratedRotation.z + accWeight * accelRotation.z
            );
        }
        else
        {
            // Use accelerometer for pitch/roll, gyro for yaw
            imuOnlyRotation = new Vector3(
                accelRotation.x, // Pitch from accelerometer
                filteredGyro.y * Mathf.Rad2Deg * deltaTime, // Yaw from gyro (integrated)
                accelRotation.z  // Roll from accelerometer
            );
        }
        
        // Apply Kalman filter to the IMU-only rotation
        fusedRotation = rotationKalman.Update(imuOnlyRotation, deltaTime);
        
        // Clamp rotation values to reasonable ranges
        fusedRotation.x = Mathf.Clamp(fusedRotation.x, -90f, 90f);  // Pitch
        fusedRotation.z = Mathf.Clamp(fusedRotation.z, -90f, 90f);  // Roll
        
        previousRotation = fusedRotation;
    }
    
    Vector3 CalculateRotationFromAccelerometer(Vector3 accel)
    {
        // Normalize accelerometer data
        Vector3 normalizedAccel = accel.normalized;
        
        // Calculate pitch and roll from accelerometer
        float pitch = Mathf.Atan2(-normalizedAccel.x, Mathf.Sqrt(normalizedAccel.y * normalizedAccel.y + normalizedAccel.z * normalizedAccel.z)) * Mathf.Rad2Deg;
        float roll = Mathf.Atan2(normalizedAccel.y, normalizedAccel.z) * Mathf.Rad2Deg;
        
        return new Vector3(pitch, 0, roll); // Yaw cannot be determined from accelerometer alone
    }
    
    void ApplySmoothing()
    {
        // Apply low-pass filter for position smoothing
        smoothedPosition = Vector3.Lerp(smoothedPosition, currentPosition, smoothingFactor);
        
        // Apply low-pass filter for rotation smoothing
        smoothedRotation = Vector3.Lerp(smoothedRotation, fusedRotation, smoothingFactor);
    }
    
    void ApplyPoseToGameObject()
    {
        // Use smoothed data for final application
        Vector3 finalPosition = smoothedPosition;
        Vector3 finalRotation = smoothedRotation;
        
        // Apply rotation reversal if enabled
        if (reverseRotations)
        {
            finalRotation = -finalRotation;
        }
        
        if (useRelativeMovement)
        {
            // Calculate relative movement from center with individual axis sensitivities
            Vector3 relativePosition = new Vector3(
                (finalPosition.x - centerPosition.x) * positionSensitivityX,
                (finalPosition.y - centerPosition.y) * positionSensitivityY,
                (finalPosition.z - centerPosition.z) * positionSensitivityZ
            );
            Vector3 relativeRotation = (finalRotation - centerRotation) * rotationSensitivity;
            
            // Apply to transform
            transform.localPosition = relativePosition;
            
            // Handle Y inversion if needed
            if (invertY)
            {
                Vector3 pos = transform.localPosition;
                pos.y = -pos.y;
                transform.localPosition = pos;
            }
            
            transform.localRotation = Quaternion.Euler(relativeRotation);
        }
        else
        {
            // Apply absolute pose with individual axis sensitivities
            Vector3 scaledPosition = new Vector3(
                finalPosition.x * positionSensitivityX,
                finalPosition.y * positionSensitivityY,
                finalPosition.z * positionSensitivityZ
            );
            transform.position = scaledPosition;
            transform.rotation = Quaternion.Euler(finalRotation * rotationSensitivity);
            
            if (invertY)
            {
                Vector3 pos = transform.position;
                pos.y = -pos.y;
                transform.position = pos;
            }
        }
        
        // Send position only to robot using inverse kinematics with fixed rotation (180, 0, 0)
        SendPositionToRobot(finalPosition);
    }
    
    Vector3 MapInputToRobotAxes(Vector3 inputPosition)
    {
        Vector3 mappedPosition = Vector3.zero;
        
        switch (inputToRobotMapping)
        {
            case AxisMapping.Direct:
                mappedPosition = new Vector3(inputPosition.x, inputPosition.y, inputPosition.z);
                break;
            case AxisMapping.SwapXY:
                mappedPosition = new Vector3(inputPosition.y, inputPosition.x, inputPosition.z);
                break;
            case AxisMapping.SwapXZ:
                mappedPosition = new Vector3(inputPosition.z, inputPosition.y, inputPosition.x);
                break;
            case AxisMapping.SwapYZ:
                mappedPosition = new Vector3(inputPosition.x, inputPosition.z, inputPosition.y);
                break;
            case AxisMapping.SwapXYAndXZ:
                mappedPosition = new Vector3(inputPosition.z, inputPosition.x, inputPosition.y);
                break;
            case AxisMapping.SwapXYAndYZ:
                mappedPosition = new Vector3(inputPosition.y, inputPosition.z, inputPosition.x);
                break;
            case AxisMapping.SwapXZAndYZ:
                mappedPosition = new Vector3(inputPosition.x, inputPosition.z, inputPosition.y);
                break;
            case AxisMapping.Custom:
                // Apply custom mapping
                mappedPosition.x = GetAxisValue(inputPosition, inputXToRobotAxis);
                mappedPosition.y = GetAxisValue(inputPosition, inputYToRobotAxis);
                mappedPosition.z = GetAxisValue(inputPosition, inputZToRobotAxis);
                break;
        }
        
        return mappedPosition;
    }
    
    float GetAxisValue(Vector3 inputPosition, RobotAxis targetAxis)
    {
        switch (targetAxis)
        {
            case RobotAxis.X: return inputPosition.x;
            case RobotAxis.Y: return inputPosition.y;
            case RobotAxis.Z: return inputPosition.z;
            default: return 0f;
        }
    }
    
    void SendPositionToRobot(Vector3 position)
    {
        if (inverseKinematics == null || jointController == null)
        {
            if (showDebugInfo)
                Debug.LogWarning("Robot components not found. Cannot send position to robot.");
            return;
        }
        
        // Map input axes to robot axes
        Vector3 mappedPosition = MapInputToRobotAxes(position);
        
        // Apply position offsets
        Vector3 offsetPosition = new Vector3(
            mappedPosition.x + positionOffsetX,
            mappedPosition.y + positionOffsetY,
            mappedPosition.z + positionOffsetZ
        );
        
        // Fixed rotation: (180, 0, 0) degrees
        Vector3 fixedRotation = new Vector3(180f, 0f, 0f);
        
        // Create desired transform with offset position and fixed rotation
        Matrix4x4 desiredTransform = inverseKinematics.CreateDesiredTransform(offsetPosition, fixedRotation);
        
        // Calculate inverse kinematics
        var solutions = inverseKinematics.CalculateIK(desiredTransform);
        
        if (solutions != null && solutions.Count > 0)
        {
            // Use solution[5] as requested
            int solutionIndex = Mathf.Min(5, solutions.Count - 1);
            float[] angles = solutions[solutionIndex];
            
            // Set the robot angles using ChangeUnityTargetAngles
            jointController.ChangeUnityTargetAngles(angles);
            
            if (showDebugInfo)
            {
                Debug.Log($"Robot IK: Original Position({position.x:F2}, {position.y:F2}, {position.z:F2}) " +
                         $"Mapped Position({mappedPosition.x:F2}, {mappedPosition.y:F2}, {mappedPosition.z:F2}) " +
                         $"Offset Position({offsetPosition.x:F2}, {offsetPosition.y:F2}, {offsetPosition.z:F2}) " +
                         $"Fixed Rotation({fixedRotation.x:F1}°, {fixedRotation.y:F1}°, {fixedRotation.z:F1}°) " +
                         $"Angles: [{angles[0]:F1}°, {angles[1]:F1}°, {angles[2]:F1}°, {angles[3]:F1}°, {angles[4]:F1}°, {angles[5]:F1}°]");
            }
        }
        else
        {
            if (showDebugInfo)
                Debug.LogWarning($"No IK solution found for offset position ({offsetPosition.x:F2}, {offsetPosition.y:F2}, {offsetPosition.z:F2}) with fixed rotation ({fixedRotation.x:F1}°, {fixedRotation.y:F1}°, {fixedRotation.z:F1}°)");
        }
    }
    
    void CalibrateCenter()
    {
        centerPosition = currentPosition;
        centerRotation = useSensorFusion ? fusedRotation : currentRotation;
        
        // Initialize smoothed values to current values
        smoothedPosition = currentPosition;
        smoothedRotation = centerRotation;
        
        // Reset gyro integration to current rotation (from accelerometer)
        Vector3 accelRotation = CalculateRotationFromAccelerometer(accelerometerData);
        gyroIntegratedRotation = accelRotation; // Start from accelerometer reading
        
        // Reset drift correction if enabled
        if (resetDriftOnCalibrate)
        {
            // Start gyro bias calibration
            StartGyroBiasCalibration();
        }
        
        // Reset Kalman filters if using Kalman fusion
        if (fusionType == SensorFusionType.Kalman)
        {
            positionKalman.Reset(currentPosition);
            rotationKalman.Reset(accelRotation); // Use accelerometer for initial rotation
            gyroKalman.Reset(Vector3.zero); // Reset gyro filter
            accelKalman.Reset(accelerometerData);
        }
        
        isCalibrated = true;
        
        if (showDebugInfo)
            Debug.Log($"✓ Calibrated center - Pos: ({centerPosition.x:F2}, {centerPosition.y:F2}, {centerPosition.z:F2}) Rot: ({accelRotation.x:F1}°, {accelRotation.y:F1}°, {accelRotation.z:F1}°)");
    }
    
    void StartGyroBiasCalibration()
    {
        gyroCalibrationSum = Vector3.zero;
        gyroCalibrationCount = 0;
        isGyroCalibrated = false;
        
        if (showDebugInfo)
            Debug.Log("🔄 Starting gyro bias calibration... Keep device still for 2 seconds");
        
        // Start calibration coroutine
        StartCoroutine(CalibrateGyroBias());
    }
    
    System.Collections.IEnumerator CalibrateGyroBias()
    {
        float calibrationTime = 2.0f; // 2 seconds of calibration
        float startTime = Time.time;
        
        while (Time.time - startTime < calibrationTime)
        {
            // Accumulate gyro readings
            gyroCalibrationSum += gyroscopeData;
            gyroCalibrationCount++;
            
            yield return null; // Wait one frame
        }
        
        // Calculate average bias
        if (gyroCalibrationCount > 0)
        {
            gyroBias = gyroCalibrationSum / gyroCalibrationCount;
            isGyroCalibrated = true;
            
            if (showDebugInfo)
                Debug.Log($"✓ Gyro bias calibrated: ({gyroBias.x:F4}, {gyroBias.y:F4}, {gyroBias.z:F4})");
        }
    }
    
    void OnDestroy()
    {
        StopUDPListener();
    }
    
    void OnApplicationQuit()
    {
        StopUDPListener();
    }
    
    void StopUDPListener()
    {
        isReceiving = false;
        
        if (udpThread != null && udpThread.IsAlive)
        {
            udpThread.Abort();
        }
        
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
        
        if (showDebugInfo)
            Debug.Log("UDP listener stopped");
    }
}
