using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class FTClient : MonoBehaviour
{
    public string host = "192.168.1.100";
    public int port = 63351;
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;
    private bool running = false;
    private float latestFz = 0f;
    public float GetFz() { return latestFz; }

    void Start()
    {
        
        StartClient();
    }

    public void StartClient()
    {
        if (running) return;
        running = true;
        clientThread = new Thread(ClientLoop);
        clientThread.IsBackground = true;
        clientThread.Start();
    }

    public void StopClient()
    {
        running = false;
        if (clientThread != null && clientThread.IsAlive)
            clientThread.Join();
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
    }

    private void ClientLoop()
    {
        try
        {
            //Debug.Log($"Connecting to {host}:{port}");
            client = new TcpClient();
            client.Connect(host, port);
            stream = client.GetStream();
            //Debug.Log($"Connected. Displaying data in console. Press StopClient() to stop.");
            byte[] buffer = new byte[1024];
            while (running)
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string data = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    data = data.Replace("(", "");
                    data = data.Replace(")", "\n");
                    //Debug.Log($"Received: {data}");

                    // Parse Fz (3rd value)
                    string[] parts = data.Split(new char[] { ',', ' ', '\t', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 3)
                    {
                        if (float.TryParse(parts[2], out float fz))
                        {
                            latestFz = fz;
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"No connection: {ex.Message}");
        }
        finally
        {
            if (stream != null)
                stream.Close();
            if (client != null)
                client.Close();
        }
    }

    void OnApplicationQuit()
    {
        StopClient();
    }
} 