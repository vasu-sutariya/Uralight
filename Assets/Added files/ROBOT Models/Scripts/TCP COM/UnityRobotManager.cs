using UnityEngine;
using UnityEngine.UI;
using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class UnityRobotManager : MonoBehaviour
{
    [Header("Robot Connection Settings")]
    public string robotIP = "192.168.1.100";
    public int dataPort = 30002;    // For reading robot state
    public int commandPort = 30003; // For sending commands
    
    [Header("UI Display")]
    public Text jointAnglesText;
    public Text toolPoseText;
    public Text robotStatusText;
    public Text connectionStatusText;
    
    [Header("Debug")]
    public bool showDebugInfo = true;
    
    // Data reading connection (port 30002)
    private TcpClient dataClient;
    private NetworkStream dataStream;
    private Thread dataThread;
    private bool isDataConnected = false;
    private bool shouldStopData = false;
    
    // Command sending connection (port 30003)
    private TcpClient commandClient;
    private NetworkStream commandStream;
    private bool isCommandConnected = false;
    
    // Data storage
    private float[] jointPositions = new float[6];
    private float[] toolPose = new float[6];
    private string robotMode = "Unknown";
    private bool isProgramRunning = false;
    private ulong timestamp = 0;
    
    // Thread-safe data access
    private readonly object dataLock = new object();
    private bool hasNewData = false;

    void Start()
    {
        ConnectToRobot();
    }

    void Update()
    {
        // Update UI on main thread
        if (hasNewData)
        {
            UpdateUI();
            hasNewData = false;
        }
    }

    public void ConnectToRobot()
    {
        // Connect to data port (30002)
        ConnectToDataPort();
        
        // Connect to command port (30003)
        ConnectToCommandPort();
    }

    private void ConnectToDataPort()
    {
        try
        {
            Debug.Log($"Attempting to connect to UR10 data port at {robotIP}:{dataPort}");
            dataClient = new TcpClient();
            dataClient.Connect(robotIP, dataPort);
            dataStream = dataClient.GetStream();
            isDataConnected = true;
            
            Debug.Log("Successfully connected to UR10 data port");
            
            // Start data reading thread
            shouldStopData = false;
            dataThread = new Thread(ReadRobotData);
            dataThread.Start();
        }
        catch (SocketException e)
        {
            Debug.LogError($"Data port connection error: {e.Message}");
        }
    }

    private void ConnectToCommandPort()
    {
        try
        {
            Debug.Log($"Attempting to connect to UR10 command port at {robotIP}:{commandPort}");
            commandClient = new TcpClient();
            commandClient.Connect(robotIP, commandPort);
            commandStream = commandClient.GetStream();
            isCommandConnected = true;
            
            Debug.Log("Successfully connected to UR10 command port");
        }
        catch (SocketException e)
        {
            Debug.LogError($"Command port connection error: {e.Message}");
        }
    }

    public void DisconnectFromRobot()
    {
        // Stop data reading
        shouldStopData = true;
        isDataConnected = false;
        
        if (dataThread != null && dataThread.IsAlive)
        {
            dataThread.Join(1000); // Wait up to 1 second for thread to stop
        }
        
        // Close data connection
        if (dataStream != null) dataStream.Close();
        if (dataClient != null) dataClient.Close();
        
        // Close command connection
        if (commandStream != null) commandStream.Close();
        if (commandClient != null) commandClient.Close();
        
        if (connectionStatusText != null)
            connectionStatusText.text = "Disconnected";
            
        Debug.Log("Disconnected from UR10");
    }

    // Command sending methods
    public void SendMoveJCommand(float[] jointPositions, float acceleration, float velocity, float t = 0f, float r = 0f)
    {
        if (!isCommandConnected || commandStream == null) 
        {
            Debug.LogWarning("Command connection not available");
            return;
        }

        try
        {
            // Convert degrees to radians
            float[] jointPositionsRad = new float[6];
            for (int i = 0; i < 6; i++)
            {
                jointPositionsRad[i] = jointPositions[i] * Mathf.Deg2Rad;
            }
            
            string moveCommand = $"movej([{string.Join(", ", jointPositionsRad)}], a={acceleration}, v={velocity}, t={t}, r={r})\n";
            byte[] commandBytes = Encoding.UTF8.GetBytes(moveCommand);
            commandStream.Write(commandBytes, 0, commandBytes.Length);
            
            Debug.Log($"Sent movej command: {moveCommand}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error sending movej command: {e.Message}");
        }
    }

    public void SendServoJCommand(float[] jointPositions, float lookaheadTime = 0.1f, int gain = 300)
    {
        if (!isCommandConnected || commandStream == null) 
        {
            Debug.LogWarning("Command connection not available");
            return;
        }

        try
        {
            // Convert degrees to radians
            float[] jointPositionsRad = new float[6];
            for (int i = 0; i < 6; i++)
            {
                jointPositionsRad[i] = jointPositions[i] * Mathf.Deg2Rad;
            }
            
            string servojCommand = $"servoj([{string.Join(", ", jointPositionsRad)}], 0, 0, 0.008, {lookaheadTime}, {gain})\n";
            byte[] commandBytes = Encoding.UTF8.GetBytes(servojCommand);
            commandStream.Write(commandBytes, 0, commandBytes.Length);
            
            //Debug.Log($"Sent servoj command: {servojCommand}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error sending servoj command: {e.Message}");
        }
    }

    public void SendMoveLCommand(float[] pose, float acceleration, float velocity, float t = 0f, float r = 0f)
    {
        if (!isCommandConnected || commandStream == null)
        {
            Debug.LogWarning("Command connection not available");
            return;
        }

        try
        {
            // Convert position from mm to meters and rotation from degrees to radians
            float[] poseMeters = new float[6];
            poseMeters[0] = pose[0] / 1000f; // X (mm to m)
            poseMeters[1] = pose[1] / 1000f; // Y (mm to m)
            poseMeters[2] = pose[2] / 1000f; // Z (mm to m)
            poseMeters[3] = pose[3] ; // Rx (deg to rad)
            poseMeters[4] = pose[4] ; // Ry (deg to rad)
            poseMeters[5] = pose[5] ; // Rz (deg to rad)

            string poseString = $"p[{string.Join(", ", poseMeters)}]";
            string moveCommand = $"movel({poseString}, a={acceleration}, v={velocity}, t={t}, r={r})\n";
            byte[] commandBytes = Encoding.UTF8.GetBytes(moveCommand);
            commandStream.Write(commandBytes, 0, commandBytes.Length);

            Debug.Log($"Sent movel command: {moveCommand}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error sending movel command: {e.Message}");
        }
    }

    // Helper functions for big-endian conversion
    private int BytesToInt32BigEndian(byte[] bytes, int offset)
    {
        if (BitConverter.IsLittleEndian)
        {
            return (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
        }
        else
        {
            return BitConverter.ToInt32(bytes, offset);
        }
    }

    private ulong BytesToUInt64BigEndian(byte[] bytes, int offset)
    {
        if (BitConverter.IsLittleEndian)
        {
            return ((ulong)bytes[offset] << 56) | ((ulong)bytes[offset + 1] << 48) | 
                   ((ulong)bytes[offset + 2] << 40) | ((ulong)bytes[offset + 3] << 32) |
                   ((ulong)bytes[offset + 4] << 24) | ((ulong)bytes[offset + 5] << 16) | 
                   ((ulong)bytes[offset + 6] << 8) | bytes[offset + 7];
        }
        else
        {
            return BitConverter.ToUInt64(bytes, offset);
        }
    }

    private double BytesToDoubleBigEndian(byte[] bytes, int offset)
    {
        if (BitConverter.IsLittleEndian)
        {
            byte[] reversed = new byte[8];
            for (int i = 0; i < 8; i++)
            {
                reversed[i] = bytes[offset + 7 - i];
            }
            return BitConverter.ToDouble(reversed, 0);
        }
        else
        {
            return BitConverter.ToDouble(bytes, offset);
        }
    }

    private void ReadRobotData()
    {
        while (!shouldStopData && isDataConnected)
        {
            try
            {
                // Read message size (4 bytes)
                byte[] sizeBuffer = new byte[4];
                int bytesRead = dataStream.Read(sizeBuffer, 0, 4);
                if (bytesRead != 4)
                {
                    Debug.LogWarning("Failed to read message size");
                    continue;
                }
                
                int messageSize = BytesToInt32BigEndian(sizeBuffer, 0);
                if (messageSize <= 0 || messageSize > 10000) // Sanity check
                {
                    Debug.LogWarning($"Invalid message size: {messageSize}");
                    continue;
                }
                
                // Read complete message
                byte[] messageBuffer = new byte[messageSize - 4]; // Subtract the 4 bytes we already read
                bytesRead = 0;
                int totalBytesRead = 0;
                
                while (totalBytesRead < messageBuffer.Length)
                {
                    bytesRead = dataStream.Read(messageBuffer, totalBytesRead, messageBuffer.Length - totalBytesRead);
                    if (bytesRead == 0) break; // Connection closed
                    totalBytesRead += bytesRead;
                }
                
                if (totalBytesRead == messageBuffer.Length)
                {
                    ParseRobotData(messageBuffer);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error reading robot data: {e.Message}");
                isDataConnected = false;
                break;
            }
        }
    }

    private void ParseRobotData(byte[] data)
    {
        try
        {
            if (data.Length < 5) return; // Need at least message type + some data
            
            int offset = 0;
            
            // Read message type (1 byte)
            byte messageType = data[offset];
            offset++;
            
            if (messageType != 16) return; // MESSAGE_TYPE_ROBOT_STATE
            
            // Parse sub-packages
            while (offset < data.Length)
            {
                if (offset + 5 > data.Length) break;
                
                // Read package size
                int packageSize = BytesToInt32BigEndian(data, offset);
                offset += 4;
                
                if (packageSize <= 0 || offset + packageSize - 4 > data.Length) break;
                
                // Read package type
                byte packageType = data[offset];
                offset++;
                
                // Parse based on package type
                switch (packageType)
                {
                    case 0: // ROBOT_MODE_DATA
                        ParseRobotModeData(data, offset, packageSize - 5);
                        break;
                    case 1: // JOINT_DATA
                        ParseJointData(data, offset, packageSize - 5);
                        break;
                    case 4: // CARTESIAN_INFO
                        ParseCartesianInfo(data, offset, packageSize - 5);
                        break;
                }
                
                offset += packageSize - 5;
            }
            
            // Mark that we have new data
            lock (dataLock)
            {
                hasNewData = true;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error parsing robot data: {e.Message}");
        }
    }

    private void ParseRobotModeData(byte[] data, int offset, int length)
    {
        if (length < 19) return;
        
        // Read timestamp (8 bytes)
        timestamp = BytesToUInt64BigEndian(data, offset);
        offset += 8;
        
        // Read boolean flags
        bool isRealRobotConnected = data[offset] != 0; offset++;
        bool isRealRobotEnabled = data[offset] != 0; offset++;
        bool isRobotPowerOn = data[offset] != 0; offset++;
        bool isEmergencyStopped = data[offset] != 0; offset++;
        bool isProtectiveStopped = data[offset] != 0; offset++;
        isProgramRunning = data[offset] != 0; offset++;
        bool isProgramPaused = data[offset] != 0; offset++;
        
        // Read robot mode (1 byte)
        byte robotModeByte = data[offset];
        offset++;
        
        // Convert robot mode to string
        robotMode = GetRobotModeString(robotModeByte);
    }

    private void ParseJointData(byte[] data, int offset, int length)
    {
        if (length < 6 * 41) return; // 6 joints * 41 bytes per joint
        
        for (int i = 0; i < 6; i++)
        {
            // q_actual (8 bytes) - joint position in radians
            float q_actual = (float)BytesToDoubleBigEndian(data, offset);
            jointPositions[i] = q_actual * Mathf.Rad2Deg; // Convert to degrees
            offset += 8;
            
            // Skip other joint data for now
            offset += 33; // q_target, qd_actual, I_actual, V_actual, T_motor, T_micro, jointMode
        }
    }

    private void ParseCartesianInfo(byte[] data, int offset, int length)
    {
        if (length < 48) return; // 6 doubles for pose
        
        for (int i = 0; i < 6; i++)
        {
            toolPose[i] = (float)BytesToDoubleBigEndian(data, offset);
            offset += 8;
        }
    }

    private string GetRobotModeString(byte mode)
    {
        switch (mode)
        {
            case 0: return "No Controller";
            case 1: return "Disconnected";
            case 2: return "Confirm Safety";
            case 3: return "Booting";
            case 4: return "Robot Ready";
            case 5: return "Idle";
            case 6: return "Backdrive";
            case 7: return "Running";
            case 8: return "Updating Firmware";
            default: return "Unknown";
        }
    }

    private void UpdateUI()
    {
        // Update connection status
        if (connectionStatusText != null)
        {
            string status = "";
            if (isDataConnected) status += "Data ✓ ";
            if (isCommandConnected) status += "Command ✓";
            if (!isDataConnected && !isCommandConnected) status = "Disconnected";
            connectionStatusText.text = status;
        }
        
        // Update joint angles text
        if (jointAnglesText != null)
        {
            string jointText = "Joint Angles (deg):\n";
            for (int i = 0; i < 6; i++)
            {
                jointText += $"Joint {i + 1}: {jointPositions[i]:F2}°\n";
            }
            jointAnglesText.text = jointText;
        }
        
        // Update tool pose text
        if (toolPoseText != null)
        {
            string toolText = "Tool Pose:\n";
            toolText += $"X: {toolPose[0]:F3} m\n";
            toolText += $"Y: {toolPose[1]:F3} m\n";
            toolText += $"Z: {toolPose[2]:F3} m\n";
            toolText += $"Rx: {toolPose[3]:F3} rad\n";
            toolText += $"Ry: {toolPose[4]:F3} rad\n";
            toolText += $"Rz: {toolPose[5]:F3} rad";
            toolPoseText.text = toolText;
        }
        
        // Update robot status text
        if (robotStatusText != null)
        {
            string statusText = $"Robot Mode: {robotMode}\n";
            statusText += $"Program Running: {(isProgramRunning ? "Yes" : "No")}\n";
            statusText += $"Timestamp: {timestamp}";
            robotStatusText.text = statusText;
        }
        
        // Debug info
        if (showDebugInfo)
        {
            Debug.Log($"Joint Angles: [{string.Join(", ", jointPositions)}]");
            Debug.Log($"Tool Pose: [{string.Join(", ", toolPose)}]");
            Debug.Log($"Robot Mode: {robotMode}, Program Running: {isProgramRunning}");
        }
    }

    void OnDestroy()
    {
        DisconnectFromRobot();
    }

    void OnApplicationQuit()
    {
        DisconnectFromRobot();
    }

    // Public methods for external access
    public float[] GetJointPositions()
    {
        lock (dataLock)
        {
            return (float[])jointPositions.Clone();
        }
    }

    public float[] GetToolPose()
    {
        lock (dataLock)
        {
            return (float[])toolPose.Clone();
        }
    }

    public string GetRobotMode()
    {
        lock (dataLock)
        {
            return robotMode;
        }
    }

    public bool IsProgramRunning()
    {
        lock (dataLock)
        {
            return isProgramRunning;
        }
    }

    public bool IsDataConnected()
    {
        return isDataConnected;
    }

    public bool IsCommandConnected()
    {
        return isCommandConnected;
    }
} 